import sys
import rclpy
import numpy as np
import tf_transformations

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rvo.utils import WorkingMode, AgentState, ManagerState, AnsiColor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from rvo_interface.msg import Goal, Goallist  # type: ignore
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Point, PoseStamped, Twist
from crazyflie_interfaces.msg import Position, FullState, VelocityWorld

from std_srvs.srv import Empty
from crazyflie_interfaces.srv import Takeoff, GoTo, Land

class agent_RVO(Node) :
    all_takeoff = False
    all_landing = False
    all_takeoff_service = None
    all_landing_service = None
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
        self.declare_parameter("NUMBER", 0)             # Number of the agent

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
        self.dt = 0.1
        self.NUMBER           = self.get_parameter("NUMBER").value

        self.landing_time = 10.0 # takes 8 seconds to land
        self.Z_SPEED      = 0.5

        self.state_publisher      = self.create_publisher(Int32, f"/agent_state{self.NUMBER}", 10)
        self.landing_command_sub  = self.create_subscription(Bool, "/landing_command", self.landing_command_callback, 10)
        self.stop_command_sub     = self.create_subscription(Bool, "/stop_command", self.on_stop_callback, 10)
        self.goal_command_sub     = self.create_subscription(Goal, f"/goal{self.NUMBER}", self.on_goal_callback, 10)

        self.drones_names = ['crazyflie{}'.format(i) for i in self.AGENTS_INDICES] # type: ignore
        self.name_to_index = {name: i for i, name in enumerate(self.drones_names)}
        self.idx = self.name_to_index[f"crazyflie{self.NUMBER}"] # type: ignore

        odom_name = {True: "/odom" , False: "/pose"}
        odom_type = {True: Odometry, False: PoseStamped}

        cmd_name = {True: "/cmd_vel", False: "/cmd_position"}
        cmd_type = {True: Twist     , False: Position}

        self.cmd_vel = False
        self.all_cmd = True

        if self.cmd_vel :
            cmd_name = {True: "/cmd_vel", False: "/cmd_vel_world"}
            cmd_type = {True: Twist     , False: VelocityWorld}

        self.odom_subscribers = []
        self.vel_publisher = self.create_publisher(Point, f"/vel_{self.NUMBER}", 10)
        self.vel_subscribers = []
        for idx, drone in enumerate(self.AGENTS_INDICES) : # type: ignore
            if drone != self.NUMBER :
                callback = lambda x, idx = idx: self.on_vel_callback(x, idx)
                self.vel_subscribers.append(self.create_subscription(Point, f"/vel_{drone}", callback, 10))

        if self.simu:
            self.twist_publisher = self.create_publisher(cmd_type[self.simu],f"crazyflie{self.NUMBER}" + cmd_name[self.simu],10)
            for i, drone in enumerate(self.drones_names):
                callback = lambda msg, idx=i: self.odom_callback(msg, idx)
                self.odom_subscribers.append(self.create_subscription(odom_type[self.simu], drone + odom_name[self.simu], callback, 10))

        elif self.simu is not None :
            global emergency
            emergency = self.create_client(Empty, 'all/emergency')
            while not emergency.wait_for_service(timeout_sec=1) :
                self.get_logger().warn("Waiting for emergency service")
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10)
            self.pose_sub = self.create_subscription(NamedPoseArray,"/poses", self.poses_callback,qos   )

            if self.cmd_vel :
                self.twist_publisher = self.create_publisher(cmd_type[self.simu], f"crazyflie{self.NUMBER}" + cmd_name[self.simu], 10)
            else :
                self.goto_service = self.create_client(GoTo, f"crazyflie{self.NUMBER}" + "/go_to")
                while not self.goto_service.wait_for_service(timeout_sec=1) :
                    self.get_logger().warn('goto service not available, waiting again... ')
            if self.all_cmd :
                if agent_RVO.all_takeoff_service is None :
                    agent_RVO.all_takeoff_service = self.create_client(Takeoff, 'all/takeoff')
                    while not agent_RVO.all_takeoff_service.wait_for_service(timeout_sec=1) :
                        self.get_logger().warn("waiting for takeoff service")
                if agent_RVO.all_landing_service is None :
                    agent_RVO.all_landing_service = self.create_client(Land, 'all/land')
                    while not agent_RVO.all_landing_service.wait_for_service(timeout_sec=1) :
                        self.get_logger().warn("waiting for landing service")
            else :
                self.takeoff_service = self.create_client(Takeoff, f"crazyflie{self.NUMBER}" + '/takeoff')
                while not self.takeoff_service.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn(
                        'takeoff service not available, waiting again... Make sure the crazyswarm is launched'
                    )
                self.landing_service = self.create_client(Land, f"crazyflie{self.NUMBER}" + "/land")
                while not self.landing_service.wait_for_service(timeout_sec=1) :
                    self.get_logger().warn('landing service not available, waiting again... ')

        self.pos = np.zeros((self.NUM_AGENTS, 3))
        self.angles = np.zeros((self.NUM_AGENTS, 3))
        self.vel = np.zeros((self.NUM_AGENTS, 3))
        self.goal = None
        self.v_opt = np.zeros(3)
        self.dist_goal = 10
        self.start_height = None

        self.timer = self.create_timer(0.5, self.RVO_callback) # type: ignore

        self.state = AgentState.TAKEOFF
        self.get_logger().info(f'Agent state : {self.state.name}')
        
        self.wait_for_time() # wait until the /clock message is correctly initialized by ros
        self.past_time = 0.
        
        self.start_time = self.get_clock().now()
        self.start_takeoff_time = None
        self.start_landing_time = None
        self.start_mission_time = None
        self.called_takeoff = False
        self.called_landing = False
        self.stabilized = False

        self.land_pose = None
        self.time_to_take_off = 3.

        self.test_velocities = []
        magnitude = (1, 2/3, 1/3)
        nb_sample = 24
        if self.DIM == 2 :
            angles = ((np.cos(2*i*np.pi/nb_sample), np.sin(2*i*np.pi/nb_sample), 0) for i in range(nb_sample))
            self.test_velocities.append(np.array((0, 0, 0)))
            for a in angles :
                for m in magnitude :
                    self.test_velocities.append(self.SPEED * m * np.array(a)) # type: ignore
        elif self.DIM == 3 :
            golden_ratio = (1+np.sqrt(5))/2
            for i in range(nb_sample) :
                theta = 2*np.pi*i*golden_ratio
                phi = np.arccos(1-2*i/nb_sample)
                v = np.array((np.cos(theta)*np.sin(phi), np.sin(phi)*np.sin(theta), np.cos(phi)))
                for m in magnitude :
                    self.test_velocities.append(m*v)

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
                self.timer = self.create_timer(self.AGENT_TIMER, self.RVO_callback) # type: ignore

    def odom_callback(self, msg, idx):
        self.pos[idx, 0] = msg.pose.pose.position.x
        self.pos[idx, 1] = msg.pose.pose.position.y
        self.pos[idx, 2] = msg.pose.pose.position.z

        if self.idx == idx :
            dir = self.goal-self.pos[idx] if self.goal is not None else np.array((0, 0, 0))
            dir_norm = np.linalg.norm(dir)
            self.v_opt = dir if dir_norm <= self.SPEED else self.SPEED/dir_norm*dir # type: ignore
            if self.DIM == 2 :
                self.v_opt[2] = 0
            self.dist_goal = np.linalg.norm(self.pos[idx]-self.goal) if self.goal is not None else 10

        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]) # type: ignore
        self.angles[idx, 0] = euler[0]
        self.angles[idx, 1] = euler[1]
        self.angles[idx, 2] = euler[2]

    def poses_callback(self, msg):
        for p in msg.poses:

            idx      = self.name_to_index.get(p.name)
            if idx is None:
                continue
            position = p.pose.position

            self.pos[idx, 0] = position.x
            self.pos[idx, 1] = position.y
            self.pos[idx, 2] = position.z

            if idx == self.idx :
                dir = self.goal-self.pos[idx] if self.goal is not None else np.array((0, 0, 0))
                dir_norm = np.linalg.norm(dir)
                self.v_opt = dir if dir_norm <= self.SPEED else self.SPEED/dir_norm*dir # type: ignore
                if self.DIM == 2 :
                    self.v_opt[2] = 0
                self.dist_goal = np.linalg.norm(self.pos[idx]-self.goal) if self.goal is not None else 10

    def on_goal_callback(self, msg) :
        if msg.has_one :
            self.goal = np.array((msg.pos.x, msg.pos.y, msg.pos.z))
        else :
            self.goal = None

        dir = self.goal - self.pos[self.idx] if self.goal is not None else np.array((0, 0, 0))
        dir_norm = np.linalg.norm(dir)
        self.v_opt = dir if dir_norm <= self.SPEED else self.SPEED/dir_norm*dir # type: ignore
        if self.DIM == 2 :
            self.v_opt[2] = 0
        self.dist_goal = np.linalg.norm(self.pos[self.idx]-self.goal) if self.goal is not None else 10

    def on_vel_callback(self, msg, idx) :
        self.vel[idx] = np.array((msg.x, msg.y, msg.z))

# ──────── Take off ───────────────────────────────────────────────────────────────────────────────

    def initiate_takeoff(self):
        if self.start_takeoff_time is None:
            self.start_takeoff_time = self.get_clock().now()
        
        self.time         = self.get_clock().now() - self.start_takeoff_time

        if self.simu:
            msg = Twist()
            msg.linear.z = np.clip((self.HOOVERING_HEIGHT - self.pos[self.idx, 2]), -self.Z_SPEED, self.Z_SPEED) # go to one meter altitude
            self.twist_publisher.publish(msg)
        else :
            self.start_height = self.pos[self.idx, 2]
            if self.all_cmd :
                if not agent_RVO.all_takeoff :
                    agent_RVO.all_takeoff = True
                    req = Takeoff.Request()
                    req.group_mask = 0
                    req.height = self.HOOVERING_HEIGHT
                    req.duration = rclpy.duration.Duration(seconds=self.time_to_take_off).to_msg() # type: ignore
                    agent_RVO.all_takeoff_service.call_async(req) # type: ignore

            else :
                if not self.called_takeoff:
                    self.called_takeoff = True
                    self.takeoff(self.HOOVERING_HEIGHT, self.time_to_take_off) # TODO: have a closer look at the height (ensure collsion avoidance)

        self.get_logger().info(f'{AnsiColor.BLUE} Taking off... Time elapsed: {self.time.nanoseconds / 1e9:.2f}s. Will finish at {self.time_to_take_off*2.0}s {AnsiColor.RESET}',throttle_duration_sec=2.0)

        return abs(self.pos[self.idx, 2]-self.HOOVERING_HEIGHT) < .2

    def takeoff(self, targetHeight, duration, groupMask=0):
        req            = Takeoff.Request()
        req.group_mask = groupMask
        req.height     = targetHeight
        req.duration   = rclpy.duration.Duration(seconds=duration).to_msg() # type: ignore
        # Wait until service call completes
        self.takeoff_service.call_async(req)

# ──────── Landing ────────────────────────────────────────────────────────────────────────────────

    def initiate_landing(self):

        if self.start_landing_time is None:
            self.start_landing_time = self.get_clock().now()

        if self.simu:
            msg = Twist()
            msg.linear.z = float(np.clip(-self.HOOVERING_HEIGHT/self.landing_time, -self.Z_SPEED, self.Z_SPEED)) if self.DIM == 2 else float(np.clip(-self.pos[self.idx, 2], -self.Z_SPEED, self.Z_SPEED)) # type: ignore
            self.twist_publisher.publish(msg)

        else :
            if self.cmd_vel :
                msg = VelocityWorld()
                msg.vel.z = float(np.clip(-self.HOOVERING_HEIGHT/self.landing_time, -self.Z_SPEED, self.Z_SPEED)) if self.DIM == 2 else float(np.clip(-self.pos[self.idx, 2], -self.Z_SPEED, self.Z_SPEED))  # type: ignore
                self.twist_publisher.publish(msg)
            else :
                if self.all_cmd :
                    if not agent_RVO.all_landing :
                        agent_RVO.all_landing = True
                        req = Land.Request()
                        req.group_mask = 0
                        req.height = self.start_height + 0.05 # type: ignore
                        req.duration = rclpy.duration.Duration(seconds=self.landing_time).to_msg() # type: ignore
                        agent_RVO.all_landing_service.call_async(req) # type: ignore
                else :
                    if not self.called_landing :
                        self.called_landing = False
                        req = Land.Request()
                        req.group_mask = 0
                        req.height = self.start_height + 0.05 # type: ignore
                        req.duration = rclpy.duration.Duration(seconds=self.landing_time).to_msg() # type: ignore
                        self.landing_service.call_async(req)
        
        return self.pos[self.idx, 2] <.15

# ──────── Apply RVO ────────────────────────────────────────────────────────────────────────────────

    def run_mission(self) :
        try :
            new_vel = RVO_loc(self.idx, self.test_velocities, self.v_opt, self.pos, self.vel)

            if self.simu:
                msg = Twist()
                msg.linear.x = float(new_vel[0])
                msg.linear.y = float(new_vel[1])
                msg.linear.z = float(np.clip(self.HOOVERING_HEIGHT - self.pos[self.idx, 2],-self.Z_SPEED,self.Z_SPEED)) if self.DIM == 2 else float(new_vel[2])
                msg.angular.z = float(np.clip(0. - self.angles[self.idx, 2],-self.SPEED,self.SPEED)) # type: ignore
                self.twist_publisher.publish(msg)
                # self.get_logger().info(f"{AnsiColor.VIOLET} vel : {new_vel[idx]}, pos : {self.pos[idx]}, goal : {self.goals[idx]}, v_opt : {self.v_opt[idx]} {AnsiColor.RESET}")
                self.vel[self.idx] = new_vel
                if self.DIM == 2 :
                    self.vel[self.idx, 2] = 0

            else :
                vel_norm = np.linalg.norm(new_vel)
                if self.dist_goal < .2 and np.array_equal(new_vel, self.v_opt) :
                    x_new = self.goal
                    if not self.cmd_vel and not self.stabilized :
                        # self.get_logger().info(f"{AnsiColor.VIOLET} Stabilizing the drone {AnsiColor.RESET}")
                        req = GoTo.Request()
                        goal = Point()
                        goal.x = float(x_new[0]) # type: ignore
                        goal.y = float(x_new[1]) # type: ignore
                        goal.z = float(self.hoover_heights[idx]) if self.DIM == 2 else float(x_new[2]) # type: ignore
                        req.goal = goal
                        req.yaw = 0.
                        duration = np.linalg.norm(x_new-self.pos[self.idx])/vel_norm if vel_norm != 0 else 0
                        duration = max(duration, 1)
                        req.duration.sec = int(duration)
                        req.duration.nanosec = int((duration%1)*1e9)
                        self.goto_service.call_async(req)
                        self.stabilized = True
                else :
                    self.stabilized = False
                    # self.get_logger().info(f"{AnsiColor.VIOLET} dist to goal : {self.dist_goal[idx]}, pos : {self.pos[idx]} , goal {self.goals[idx]} :  {AnsiColor.RESET}")
                    MINIMAL_SIZE_STEP = .05 if self.dist_goal < .4 else .1
                    # MINIMAL_SIZE_STEP = vel_norm
                    dp = new_vel
                    dp_norm = np.linalg.norm(dp)
                    if self.cmd_vel :
                        MINIMAL_SIZE_STEP = .2
                    if dp_norm < MINIMAL_SIZE_STEP and np.array_equal(new_vel, self.v_opt) :
                        dp = dp/dp_norm * MINIMAL_SIZE_STEP
                        dp_norm = MINIMAL_SIZE_STEP
                    x_new = self.pos[self.idx] + dp
                if self.cmd_vel :
                    msg = VelocityWorld()
                    msg.vel.x = float(new_vel[0])
                    msg.vel.y = float(new_vel[1])
                    msg.vel.z = float(np.clip(self.HOOVERING_HEIGHT - self.pos[self.idx, 2],-self.Z_SPEED,self.Z_SPEED)) if self.DIM == 2 else float(new_vel[2])
                    msg.yaw_rate = float(np.clip(0. - self.angles[idx, 2],-self.SPEED,self.SPEED)) # type: ignore
                    self.twist_publisher.publish(msg)
                    # self.get_logger().info(f"{AnsiColor.VIOLET} vel : {new_vel[idx]}, pos : {self.pos[idx]}, goal : {self.goals[idx]}, v_opt : {self.v_opt[idx]} {AnsiColor.RESET}")
                elif not self.stabilized :
                    req = GoTo.Request()
                    goal = Point()
                    goal.x = float(x_new[0]) # type: ignore
                    goal.y = float(x_new[1]) # type: ignore
                    goal.z = float(self.hoover_heights[idx]) if self.DIM == 2 else float(x_new[2]) # type: ignore
                    req.goal = goal
                    req.yaw = 0.
                    duration = dp_norm/vel_norm if vel_norm != 0 else 0 # type: ignore
                    duration = max(duration, 1)
                    req.duration.sec = int(duration)
                    req.duration.nanosec = int((duration%1)*1e9)
                    self.goto_service.call_async(req)

                self.vel[self.idx] = new_vel
                if self.DIM == 2 :
                    self.vel[self.idx, 2] = 0

            msg = Point()
            msg.x = float(self.vel[self.idx, 0])
            msg.y = float(self.vel[self.idx, 1])
            msg.z = float(self.vel[self.idx, 2])
            self.vel_publisher.publish(msg)
        except KeyboardInterrupt :
            pass

# ──────── Main loop ──────────────────────────────────────────────────────────────────────────────

    def RVO_callback(self) :
        self.time      = self.get_clock().now() - self.start_time
        self.state_publisher.publish(Int32(data=int(self.state)))

        self.get_logger().info(f"{AnsiColor.BLUE}crazyflie {self.NUMBER} STATE: {self.state.name} {AnsiColor.RESET}", throttle_duration_sec=3.0)

        if self.land_pose is None:
            self.land_pose = self.pos.copy()

        match self.state :
            case AgentState.TAKEOFF :
                is_finished = self.initiate_takeoff()
                if is_finished:
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

def is_in_vo(idx, other_idx, v_test, pos) :
    TAU = 350
    RADIUS = .1
    MARGIN = 0
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
    if (np.linalg.norm(dp-lambda_*v) <= 2*RADIUS + MARGIN) :
        if lambda_ <= v_norm :
            if dp@v > 0 :
                return np.inf
            return 0
        return v_norm/lambda_
    return 0

def RVO_loc(idx, test_velocities, v_opt, pos, vel) :
    DIST_DETECT = 3
    v_tests = [v for v in test_velocities] + [v_opt]
    v_tests.sort(key = lambda x : np.linalg.norm(v_opt - x))
    costs = [0 for _ in v_tests]
    for other_idx, pos_other in enumerate(pos) :
        if (other_idx != idx and np.linalg.norm(pos[idx] - pos_other) <= DIST_DETECT) :
            for i, v in enumerate(v_tests) :
                res = is_in_vo(idx, other_idx, 2*v-vel[idx]-vel[other_idx], pos)
                costs[i] += res # type: ignore

    # self.get_logger().info(f"{AnsiColor.VIOLET} cost : {costs} {AnsiColor.RESET}")
    return v_tests[np.argmin(costs)]

def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    if not agents.simu :
        empty = Empty.Request()
        emergency.call_async(empty)
    agents.destroy_node()
    if rclpy.utilities.ok() : # type: ignore
        rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=4)
    global agents
    agents = agent_RVO()

    executor.add_node(agents)
    try:
        executor.spin()
    except KeyboardInterrupt :
        pass # Go to finally to clean, just avoid displaying the error
    finally:
        if not agents.simu :
            empty = Empty.Request()
            emergency.call_async(empty)
        agents.destroy_node()
        if rclpy.utilities.ok() : # type: ignore
            rclpy.shutdown()

if __name__ == "__main__":
    main()
