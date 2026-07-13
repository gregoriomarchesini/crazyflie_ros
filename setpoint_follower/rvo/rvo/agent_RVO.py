import sys
import rclpy
import signal
import numpy as np
import tf_transformations

from rclpy.node import Node
from multiprocessing import Pool
from rclpy.executors import MultiThreadedExecutor
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rvo.utils import WorkingMode, AgentState, ManagerState, AnsiColor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool
from rcl_interfaces.msg import Parameter
from geometry_msgs.msg import Point, PoseStamped, Twist
from rvo_interface.msg import Goal, Goallist  # type: ignore
from crazyflie_interfaces.msg import Position, FullState, VelocityWorld

from std_srvs.srv import Empty
from rcl_interfaces.srv import SetParameters
from crazyflie_interfaces.srv import Takeoff, GoTo, Land, NotifySetpointsStop

class agent_RVO(Node) :
    def __init__(self) :
        super().__init__("drone")

        self.declare_parameter("backend", "")           # simu or real
        self.declare_parameter("AGENTS_INDICES",[1])    # list if all the index of agents
        self.declare_parameter("DIM", 10)               # dimension
        self.declare_parameter("NUM_AGENTS", 10)        # number of agents
        self.declare_parameter("SPEED", 0.4)            # max speed
        self.declare_parameter("AGENT_TIMER", 0.1)      # frequency
        self.declare_parameter("HOOVERING_HEIGHT",1.0)  # alitude at which the drones move nominally
        self.declare_parameter("COMM_DISTANCE", 1.0)    # distance at which we take other into account

        backend  = self.get_parameter("backend").value
        self.simu = None
        if backend == "sim":
            self.simu = True
        elif backend == "hardware":
            self.simu = False

        self.DIM              = self.get_parameter("DIM").value
        self.SPEED            = self.get_parameter("SPEED").value
        self.AGENT_TIMER      = self.get_parameter("AGENT_TIMER").value
        self.HOOVERING_HEIGHT = self.get_parameter("HOOVERING_HEIGHT").value
        self.COMM_DISTANCE    = self.get_parameter("COMM_DISTANCE").value
        if (not self.COMM_DISTANCE) :
            self.COMM_DISTANCE = np.inf
        self.AGENTS_INDICES   = self.get_parameter("AGENTS_INDICES").value
        self.NUM_AGENTS       = len(self.AGENTS_INDICES) # type: ignore
        self.dt = 2*self.AGENT_TIMER # type: ignore

        self.landing_time = 10.0 # takes 8 seconds to land
        self.Z_SPEED      = 0.5

        self.fast_cb_group = ReentrantCallbackGroup()
        self.rvo_cb_group = MutuallyExclusiveCallbackGroup()

        self.state_publisher      = self.create_publisher(Int32, "/agent_state", 10, callback_group=self.fast_cb_group)
        self.landing_command_sub  = self.create_subscription(Bool, "/landing_command", self.landing_command_callback, 10, callback_group=self.fast_cb_group)
        self.stop_command_sub     = self.create_subscription(Bool, "/stop_command", self.on_stop_callback, 10, callback_group=self.fast_cb_group)
        self.goal_command_sub     = self.create_subscription(Goallist, "/goals", self.on_goal_callback, 10, callback_group= self.fast_cb_group)

        self.drones_names = ['crazyflie{}'.format(i) for i in self.AGENTS_INDICES] # type: ignore
        self.name_to_index = {name: i for i, name in enumerate(self.drones_names)}

        odom_name = {True: "/odom" , False: "/pose"}
        odom_type = {True: Odometry, False: PoseStamped}

        cmd_name = {True: "/cmd_vel", False: "/cmd_position"}
        cmd_type = {True: Twist     , False: Position}

        self.cmd_vel = False
        self.cmd_all = False
        self.time_between_command = 0.5
        if self.cmd_vel :
            cmd_name = {True: "/cmd_vel", False: "/cmd_velocity_world"}
            cmd_type = {True: Twist     , False: VelocityWorld}

        self.odom_subscribers = []
        self.twist_publishers = []
        self.takeoff_services = []
        self.landing_services = []
        self.start_height = []
        self.command_time = [self.time_between_command*i/self.NUM_AGENTS for i in range(self.NUM_AGENTS)]
        if self.cmd_all and not self.simu:
            self.takeoff_services = self.create_client(Takeoff, 'all/takeoff')
            while not self.takeoff_services.wait_for_service(timeout_sec=1) :
                self.get_logger().warn("waiting for takeoff service")
            self.landing_services = self.create_client(Land, 'all/land')
            while not self.landing_services.wait_for_service(timeout_sec=1) :
                self.get_logger().warn("waiting for landing service")
        if not self.simu:
            self.hoover_heights   = [self.HOOVERING_HEIGHT  for idx in range(self.NUM_AGENTS)]
        else:
            self.hoover_heights   = [self.HOOVERING_HEIGHT + 0*idx for idx in range(self.NUM_AGENTS)] # type: ignore

        if self.simu:
            for i, drone in enumerate(self.drones_names):
                callback = lambda msg, idx=i: self.odom_callback(msg, idx)
                self.odom_subscribers.append(self.create_subscription(odom_type[self.simu], drone + odom_name[self.simu], callback, 10))
                self.twist_publishers.append(self.create_publisher(cmd_type[self.simu],drone + cmd_name[self.simu],10))

        elif self.simu is not None :
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10)
            self.pose_sub = self.create_subscription(NamedPoseArray,"/poses", self.poses_callback,qos   )

            for drone in self.drones_names:

                if self.cmd_vel :
                    self.twist_publishers.append(
                        self.create_publisher(cmd_type[self.simu], drone + cmd_name[self.simu], 10)
                    )

                else :
                    goto_service = self.create_client(GoTo, drone + "/go_to")

                    while not goto_service.wait_for_service(timeout_sec=1) :
                        self.get_logger().warn('goto service not available, waiting again... Make sure the crazyswarm is launched')

                    self.twist_publishers.append(goto_service)

                if not self.cmd_all :

                    takeoffService = self.create_client(Takeoff, drone + '/takeoff')

                    while not takeoffService.wait_for_service(timeout_sec=1.0):
                        self.get_logger().warn(
                            'takeoff service not available, waiting again... '
                        )
                    self.takeoff_services.append(takeoffService) # type: ignore

                    landingService = self.create_client(Land, drone + '/land')

                    while not landingService.wait_for_service(timeout_sec=1.0):
                        self.get_logger().warn(
                            'landing service not available, waiting again... '
                        )
                    self.landing_services.append(landingService) # type: ignore

        if not self.simu :
            global emergency
            emergency = self.create_client(Empty, 'all/emergency')
            while not emergency.wait_for_service(timeout_sec=1) :
                self.get_logger().warn("Waiting for emergency service")

        self.pos = np.zeros((self.NUM_AGENTS, 3))
        self.angles = np.zeros((self.NUM_AGENTS, 3))
        self.vel = np.zeros((self.NUM_AGENTS, 3))
        self.goals = [None for _ in range(self.NUM_AGENTS)]
        self.v_opt = np.zeros((self.NUM_AGENTS, 3))
        self.dist_goal = [10 for _ in range(self.NUM_AGENTS)]

        self.timer = self.create_timer(self.AGENT_TIMER, self.RVO_callback, callback_group= self.rvo_cb_group) # type: ignore

        self.state = AgentState.TAKEOFF
        self.get_logger().info(f'Agent state : {self.state.name}')
        
        self.wait_for_time() # wait until the /clock message is correctly initialized by ros
        self.past_time = 0.
        
        self.start_time = self.get_clock().now()
        self.start_takeoff_time = None
        self.called_takeoff = False
        self.called_landing = False
        self.start_landing_time = None
        self.start_mission_time = None
        self.stabilized = [False for _ in range(self.NUM_AGENTS)]

        self.land_pose = None
        self.time_to_take_off = 3.

        self.test_velocities = []
        self.magnitudes = (1, 2/3, 1/3)
        nb_sample = 24
        if self.DIM == 2 :
            angles = ((np.cos(2*i*np.pi/nb_sample), np.sin(2*i*np.pi/nb_sample), 0) for i in range(nb_sample))
            self.test_velocities.append(np.array((0, 0, 0)))
            for a in angles :
                for m in self.magnitudes :
                    self.test_velocities.append(self.SPEED * m * np.array(a)) # type: ignore
        elif self.DIM == 3 :
            golden_ratio = (1+np.sqrt(5))/2
            for i in range(nb_sample) :
                theta = 2*np.pi*i*golden_ratio
                phi = np.arccos(1-2*i/nb_sample)
                v = np.array((np.cos(theta)*np.sin(phi), np.sin(phi)*np.sin(theta), np.cos(phi)))
                for m in self.magnitudes :
                    self.test_velocities.append(m*v)
        self.min_dist = np.inf

# ──────── subscription function ──────────────────────────────────────────────────────────────────

    def landing_command_callback(self, msg):
        if msg.data == True: 
            if self.state != AgentState.LANDING:
                self.get_logger().info(f'{AnsiColor.BLUE} Received landing command. Switching to LANDING state... {AnsiColor.RESET}')
                self.state = AgentState.LANDING

    def on_stop_callback(self, msg):
        if msg.data == True:
            if self.state != AgentState.STOP:
                self.get_logger().info(f'{AnsiColor.RED} Received STOP command... {AnsiColor.RESET}')
                self.state = AgentState.STOP
                self.timer.cancel()
        elif msg.data == False:
            if self.state == AgentState.STOP:
                self.get_logger().info(f'{AnsiColor.GREEN} Resuming from STOP command... {AnsiColor.RESET}')
                self.state = AgentState.READY
                self.timer = self.create_timer(self.AGENT_TIMER, self.RVO_callback, callback_group= self.rvo_cb_group) # type: ignore

    def odom_callback(self, msg, idx):
        self.pos[idx, 0] = msg.pose.pose.position.x
        self.pos[idx, 1] = msg.pose.pose.position.y
        self.pos[idx, 2] = msg.pose.pose.position.z

        dir = self.goals[idx]-self.pos[idx] if self.goals[idx] is not None else np.array((0, 0, 0))
        dir_norm = np.linalg.norm(dir)
        self.v_opt[idx] = dir if dir_norm <= self.SPEED else self.SPEED/dir_norm*dir # type: ignore
        if self.DIM == 2 :
            self.v_opt[idx][2] = 0

        q = msg.pose.pose.orientation

        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]) # type: ignore
        self.angles[idx, 0] = euler[0]
        self.angles[idx, 1] = euler[1]
        self.angles[idx, 2] = euler[2]
        self.dist_goal[idx] = np.linalg.norm(self.pos[idx]-self.goals[idx]) if self.goals[idx] is not None else 10 # type: ignore

    def poses_callback(self, msg):
        for p in msg.poses:

            idx      = self.name_to_index.get(p.name)
            if idx is None:
                continue
            position = p.pose.position

            self.pos[idx, 0] = position.x
            self.pos[idx, 1] = position.y
            self.pos[idx, 2] = position.z

            dir = self.goals[idx]-self.pos[idx] if self.goals[idx] is not None else np.array((0, 0, 0))
            dir_norm = np.linalg.norm(dir)
            self.v_opt[idx] = dir if dir_norm <= self.SPEED else self.SPEED/dir_norm*dir # type: ignore
            if self.DIM == 2 :
                self.v_opt[idx][2] = 0
            self.dist_goal[idx] = np.linalg.norm(self.pos[idx]-self.goals[idx]) if self.goals[idx] is not None else 10 # type: ignore
            # self.get_logger().info(f"{AnsiColor.VIOLET} receiving pos {self.pos[idx]} for drone {p.name} for goal {self.goals[idx]} {AnsiColor.RESET}")

    def on_goal_callback(self, msg) :
        # self.get_logger().info(f"{AnsiColor.VIOLET} receiving goals {AnsiColor.RESET}")
        for goal in msg.goals :
            # self.get_logger().info(f"{AnsiColor.VIOLET} receiving goal : has_one : {goal.has_one}, index : {goal.index}, pos : {goal.pos.x, goal.pos.y, goal.pos.z} {AnsiColor.RESET}")
            name = "crazyflie" + str(goal.index)
            if goal.has_one :
                self.stabilized[self.name_to_index[name]] = False
                self.goals[self.name_to_index[name]] = np.array((goal.pos.x, goal.pos.y, goal.pos.z)) # type: ignore
                if self.DIM == 2 :
                    self.goals[self.name_to_index[name]][2] = self.hoover_heights[self.name_to_index[name]] # type: ignore
            else :
                self.goals[self.name_to_index[name]] = None

            dir = self.goals[self.name_to_index[name]] - self.pos[self.name_to_index[name]] if self.goals[self.name_to_index[name]] is not None else np.array((0, 0, 0))
            dir_norm = np.linalg.norm(dir)
            self.v_opt[self.name_to_index[name]] = dir if dir_norm <= self.SPEED else self.SPEED/dir_norm*dir # type: ignore
            if self.DIM == 2 :
                self.v_opt[self.name_to_index[name]][2] = 0

# ──────── Take off ───────────────────────────────────────────────────────────────────────────────

    def initiate_takeoff(self):
        if self.start_takeoff_time is None:
            self.start_takeoff_time = self.get_clock().now()
        
        self.time         = self.get_clock().now() - self.start_takeoff_time

        if self.simu:
            self.start_height = 0
            for idx, publisher in enumerate(self.twist_publishers):
                msg = Twist()
                msg.linear.z = np.clip((self.hoover_heights[idx] - self.pos[idx, 2]), -self.Z_SPEED, self.Z_SPEED) # go to one meter altitude
                publisher.publish(msg)
        else :
            if not self.called_takeoff:
                self.called_takeoff = True
                if self.cmd_all :
                    self.start_height = self.pos[0, 2]
                    req = Takeoff.Request()
                    req.group_mask = 0
                    req.height = self.HOOVERING_HEIGHT
                    req.duration = rclpy.duration.Duration(seconds=self.time_to_take_off).to_msg() # type: ignore
                    self.takeoff_services.call_async(req) # type: ignore
                else :
                    for idx, publisher in enumerate(self.twist_publishers):
                        self.start_height.append(self.pos[idx, 2]) # type: ignore
                        self.takeoff(self.hoover_heights[idx], self.time_to_take_off, idx) # TODO: have a closer look at the height (ensure collsion avoidance)

        self.get_logger().info(f'{AnsiColor.BLUE} Taking off... Time elapsed: {self.time.nanoseconds / 1e9:.2f}s. Will finish at {self.time_to_take_off*2.0}s {AnsiColor.RESET}',throttle_duration_sec=2.0)

        take_off_finished = True
        for idx, pos in enumerate(self.pos) :
            if abs(pos[2]-self.hoover_heights[idx]) > .2 :
                take_off_finished = False
                break

        return take_off_finished

    def takeoff(self, targetHeight, duration, idx, groupMask=0):
        req            = Takeoff.Request()
        req.group_mask = groupMask
        req.height     = targetHeight
        req.duration   = rclpy.duration.Duration(seconds=duration).to_msg() # type: ignore
        # Wait until service call completes
        self.takeoff_services[idx].call_async(req) # type: ignore

# ──────── Landing ────────────────────────────────────────────────────────────────────────────────

    def initiate_landing(self):

        if self.start_landing_time is None:
            self.start_landing_time = self.get_clock().now()

        self.time         = self.get_clock().now() - self.start_landing_time
        time_sec = self.time.nanoseconds / 1e9

        if self.simu:
            for idx, publisher in enumerate(self.twist_publishers):
                msg = Twist()
                msg.linear.z = float(np.clip(-self.hoover_heights[idx]/self.landing_time, -self.Z_SPEED, self.Z_SPEED)) if self.DIM == 2 else float(np.clip(-self.pos[idx, 2], -self.Z_SPEED, self.Z_SPEED)) # type: ignore
                publisher.publish(msg)

        else :
            if self.cmd_vel :
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = VelocityWorld()
                    msg.linear.z = float(np.clip(-self.hoover_heights[idx]/self.landing_time, -self.Z_SPEED, self.Z_SPEED)) if self.DIM == 2 else float(np.clip(-self.pos[idx, 2], -self.Z_SPEED, self.Z_SPEED)) # type: ignore
                    publisher.publish(msg)
            else :
                if not self.called_landing :
                    self.called_landing = True
                    if self.cmd_all :
                        # self.get_logger().info(f"{AnsiColor.VIOLET} landing called at height {self.start_height} {AnsiColor.RESET}")
                        req = Land.Request()
                        req.group_mask = 0
                        req.height = self.start_height + 0.05 # pyright: ignore[reportOperatorIssue]
                        req.duration = rclpy.duration.Duration(seconds=self.landing_time).to_msg() # type: ignore
                        self.landing_services.call_async(req) # type: ignore
                    else :
                        for idx, srv in enumerate(self.landing_services) : # pyright: ignore[reportArgumentType]
                            req = Land.Request()
                            req.group_mask = 0
                            req.height = self.start_height[idx] + 0.05 # pyright: ignore[reportIndexIssue]
                            req.duration = rclpy.duration.Duration(seconds=self.landing_time).to_msg() # pyright: ignore[reportAttributeAccessIssue]
                            srv.call_async(req)

        landing_finished = True
        for pos in self.pos :
            if pos[2] > 0.15 :
                landing_finished = False
                break

        return landing_finished

# ──────── Apply RVO ──────────────────────────────────────────────────────────────────────────────

    def run_mission(self) :
        try :
            time_sec = self.time.nanoseconds / 1e9
            if self.simu :
                new_vel = pool.starmap(RVO_loc, [(idx, self.test_velocities, self.v_opt, self.pos, self.vel, self.stabilized, self.start_height) for idx in range(self.NUM_AGENTS)])
            else :
                compute_idx = [idx for idx in range(self.NUM_AGENTS) if time_sec >= self.command_time[idx]]
                new_vel = pool.starmap(RVO_loc, [(idx, self.test_velocities, self.v_opt, self.pos, self.vel, self.stabilized, self.start_height if self.cmd_all else min(self.start_height)) for idx in compute_idx]) # type: ignore

            if self.simu:
                for idx, publisher in enumerate(self.twist_publishers):
                    msg = Twist()
                    msg.linear.x = float(new_vel[idx][0])
                    msg.linear.y = float(new_vel[idx][1])
                    msg.linear.z = float(np.clip(self.hoover_heights[idx] - self.pos[idx, 2],-self.Z_SPEED,self.Z_SPEED)) if self.DIM == 2 else float(new_vel[idx][2])
                    msg.angular.z = float(np.clip(0. - self.angles[idx, 2],-self.SPEED,self.SPEED)) # type: ignore
                    publisher.publish(msg)
                    if (self.dist_goal[idx] < .1 and np.array_equal(new_vel[idx], self.v_opt[idx])) :
                        self.stabilized[idx] = True
                    else :
                        self.stabilized[idx] = False
                    # self.get_logger().info(f"{AnsiColor.VIOLET} vel : {new_vel[idx]}, pos : {self.pos[idx]}, goal : {self.goals[idx]}, v_opt : {self.v_opt[idx]} {AnsiColor.RESET}")
                    self.vel[idx] = new_vel[idx]
                    if self.DIM == 2 :
                        self.vel[idx][2] = 0
            else :
                for i, idx in enumerate(compute_idx): # type: ignore
                    self.command_time[idx] = time_sec + self.time_between_command
                    publisher = self.twist_publishers[idx]
                    vel_norm = np.linalg.norm(new_vel[i])
                    if (self.dist_goal[idx] < self.SPEED or self.stabilized[idx]) and np.array_equal(new_vel[i], self.v_opt[idx]) : # type: ignore
                        x_new = self.goals[idx]
                        if not self.cmd_vel and not self.stabilized[idx] :
                            # self.get_logger().info(f"{AnsiColor.VIOLET} Stabilizing the drone {AnsiColor.RESET}")
                            req = GoTo.Request()
                            goal = Point()
                            goal.x = float(x_new[0]) # type: ignore
                            goal.y = float(x_new[1]) # type: ignore
                            goal.z = float(self.hoover_heights[idx]) if self.DIM == 2 else float(x_new[2]) # type: ignore
                            req.goal = goal
                            req.yaw = 0.
                            duration = 2*np.linalg.norm(x_new-self.pos[idx])/vel_norm if vel_norm != 0 else 0
                            duration = max(duration, 1)
                            req.duration.sec = int(duration)
                            req.duration.nanosec = int((duration%1)*1e9)
                            self.twist_publishers[idx].call_async(req)
                            self.stabilized[idx] = True
                    else :
                        self.stabilized[idx] = False
                        dp = new_vel[i]
                        x_new = self.pos[idx] + dp
                    if self.cmd_vel :
                        msg = VelocityWorld()
                        msg.vel.x = float(new_vel[idx][0])
                        msg.vel.y = float(new_vel[idx][1])
                        msg.vel.z = float(np.clip(self.hoover_heights[idx] - self.pos[idx, 2],-self.Z_SPEED,self.Z_SPEED)) if self.DIM == 2 else float(new_vel[idx][2])
                        msg.yaw_rate = float(np.clip(0. - self.angles[idx, 2],-self.SPEED,self.SPEED)) # pyright: ignore[reportOptionalOperand]
                        publisher.publish(msg)
                    elif not self.stabilized[idx] :
                        req = GoTo.Request()
                        goal = Point()
                        goal.x = float(x_new[0]) # type: ignore
                        goal.y = float(x_new[1]) # type: ignore
                        goal.z = float(self.hoover_heights[idx]) if self.DIM == 2 else float(x_new[2]) # type: ignore
                        req.goal = goal
                        req.yaw = 0.
                        duration = 2 if vel_norm != 0 else 0
                        duration = max(duration, 1)
                        req.duration.sec = int(duration)
                        req.duration.nanosec = int((duration%1)*1e9)
                        # self.get_logger().info(f"{AnsiColor.VIOLET} duration : {duration} {AnsiColor.RESET}")
                        self.twist_publishers[idx].call_async(req)

                    self.vel[idx] = np.array([0, 0, 0]) if self.stabilized[idx] and self.dist_goal[idx] < .05 else new_vel[i]
                    if self.DIM == 2 :
                        self.vel[idx][2] = 0
        except KeyboardInterrupt :
            pass # No need to close the pool, managed by the signal_handler, it is juste to avoid all the error messages

# ──────── Main loop ──────────────────────────────────────────────────────────────────────────────

    def RVO_callback(self) :
        self.time      = self.get_clock().now() - self.start_time
        self.state_publisher.publish(Int32(data=int(self.state)))

        self.get_logger().info(f"{AnsiColor.BLUE}MPC STATE: {self.state.name} {AnsiColor.RESET}", throttle_duration_sec=3.0)

        if self.land_pose is None:
            self.land_pose = self.pos.copy()

        match self.state :
            case AgentState.TAKEOFF :
                is_finished = self.initiate_takeoff()
                if is_finished:
                    self.command_time = [self.time.nanoseconds/1e9 + t for t in self.command_time]
                    self.state = AgentState.READY
                    self.get_logger().info(f'{AnsiColor.BLUE} Switching to READY state. Starting MPC mission...{AnsiColor.RESET}')
                    self.start_time = self.get_clock().now()

            case AgentState.LANDING:
                is_finished = self.initiate_landing()
                if is_finished:
                    self.get_logger().info(f'{AnsiColor.BLUE} Landing completed. Shutting down node...{AnsiColor.RESET}', throttle_duration_sec=5.0)
                    # self.get_logger().info(f'{AnsiColor.BLUE} Min dist between two drones : {self.min_dist} {AnsiColor.RESET}', throttle_duration_sec=5.0)

            case AgentState.READY:
                self.run_mission()

# ──────── Utils ──────────────────────────────────────────────────────────────────────────────────

    def compute_min_dist(self) :
        for i in range(self.NUM_AGENTS) :
            for j in range(i+1, self.NUM_AGENTS) :
                self.min_dist = min(self.min_dist, np.linalg.norm(self.pos[i]-self.pos[j]))

    def wait_for_time(self):
        while self.get_clock().now().nanoseconds == 0:
            self.get_logger().info("Waiting for valid ROS time...")
            rclpy.spin_once(self, timeout_sec=0.1)

def is_in_vo(idx, other_idx, v_test, pos, stabilized) :
    if stabilized[idx] and stabilized[other_idx] :
        return 0
    TAU = 3
    RADIUS = (.15 if not stabilized[other_idx] else .1) if agents.DIM == 2 else (.2 if not stabilized[other_idx] else .1)
    v_norm = np.linalg.norm(v_test)
    if v_norm == 0 :
        return 0
    v = v_test/v_norm
    dp = pos[other_idx] - pos[idx]
    lambda_ = dp @ v
    if lambda_ < 0 :
        lambda_ = 0
    elif lambda_ > TAU * v_norm :
        lambda_ = TAU*v_norm
    if (np.linalg.norm(dp-lambda_*v) <= 2*RADIUS) :
        if lambda_ <= agents.time_between_command * v_norm :
            if dp@v > 0 :
                return np.inf
            return 0
        return v_norm/lambda_
    return 0

def RVO_loc(idx, test_velocities, v_opt, pos, vel, stabilized, floor) :
    DIST_DETECT = 3
    v_tests = [v for v in test_velocities] + [v_opt[idx]]
    v_tests.sort(key = lambda x : np.linalg.norm(v_opt[idx] - x))
    costs = [0 for _ in v_tests]
    for other_idx, pos_other in enumerate(pos) :
        if (other_idx != idx and np.linalg.norm(pos[idx] - pos_other) <= DIST_DETECT) :
            for i, v in enumerate(v_tests) :
                res = np.inf if pos[idx, 2]-floor < -v[2]*agents.time_between_command else is_in_vo(idx, other_idx, 2*v-vel[idx]-vel[other_idx], pos, stabilized)
                costs[i] += res # type: ignore
    return v_tests[np.argmin(costs)]

def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    if not agents.simu :
        empty = Empty.Request()
        emergency.call_async(empty)
    agents.destroy_node()
    if rclpy.utilities.ok() : # type: ignore
        rclpy.shutdown()
    pool.close()
    pool.join()
    sys.exit(0)

def init_worker():
    signal.signal(signal.SIGINT, signal.SIG_IGN)

def main(args=None):
    rclpy.init(args=args)

    global agents
    executor = MultiThreadedExecutor(num_threads=4)
    agents = agent_RVO()

    global pool
    pool = Pool(10, init_worker)

    executor.add_node(agents)
    try:
        executor.spin()
    except KeyboardInterrupt :
        pass # Go to finally to clean, just avoid displaying the error
    finally:
        pool.close()
        pool.join()
        if not agents.simu :
            empty = Empty.Request()
            emergency.call_async(empty)
        agents.destroy_node()
        if rclpy.utilities.ok() : # type: ignore
            rclpy.shutdown()

if __name__ == "__main__":
    main()
