import sys
import rclpy
import signal
import numpy as np

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
# from scipy.optimize import minimize, LinearConstraint
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rvo.utils import WorkingMode, AgentState, ManagerState, AnsiColor, Task
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Bool
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped, Point
from rvo_interface.msg import Goal, Goallist # type: ignore
from motion_capture_tracking_interfaces.msg import NamedPoseArray

class Manager(Node) :
    def __init__(self):
        super().__init__('manager',allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides = True)
        
        # Parameters are declared from indie the yaml file. Since we have set 
        # automatically_declare_parameters_from_overrides = true, then all the parameters inside
        # the yaml file will be already automatically declared.

        # self.declare_parameter('robot_prefix', '/crazyflie')
        # self.declare_parameter("SYSTEM", 2)
        # self.declare_parameter("DIM", 2)
        # self.declare_parameter("NUM_AGENTS", 10)
        # self.declare_parameter("MANAGER_TIMER", 0.1)
        # self.declare_parameter("SETPOINT_TIMER", 0.1)
        # self.declare_parameter("COMM_DISTANCE", 1.1)
        # self.declare_parameter("BOX_WEIGHT", 10)
        
        backend            = self.get_parameter("backend").value
        if backend == "sim":
            self.SYSTEM = WorkingMode.SIM
        elif backend == "hardware":
            self.SYSTEM = WorkingMode.REAL


        self.AGENTS_INDICES = self.get_parameter("AGENTS_INDICES").value
        self.DIM            = self.get_parameter("DIM").value
        self.MANAGER_TIMER  = self.get_parameter("MANAGER_TIMER").value
        self.COMM_DISTANCE  = self.get_parameter("COMM_DISTANCE").value
        self.NUM_AGENTS     = len(self.AGENTS_INDICES) # type: ignore

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.drones_names = ['crazyflie{}'.format(i) for i in self.AGENTS_INDICES] # type: ignore
        self.name_to_index = {name: i for i, name in enumerate(self.drones_names)}

        ##
        self.start_time = None
        self.ready_count = 0
        self.current_tasks = []

        #setup the tasks that are to be done
        self.tasks, self.periods = self._load_tasks()
        self.get_logger().info(f"Loaded {len(self.tasks)} tasks")

        self.recalc_times    = self.recalculate_at()
        self.triggered_times = set()

        #setup all the subscribers and publishers
        
        self.odom_subscribers        = []
        self.goal_pub                = self.create_publisher(Goallist, "/goals", 10)

        self.landing_command_pub  = self.create_publisher(Bool, "/landing_command", 10)
        self.agent_state_sub      = self.create_subscription(Int32, "/agent_state", self.on_agent_message, 10)
        self.pos = np.zeros((self.NUM_AGENTS, 3))

        odom_name = {WorkingMode.SIM: "/odom", WorkingMode.REAL: "/pose"}
        odom_type = {WorkingMode.SIM: Odometry, WorkingMode.REAL: PoseStamped}

        
        if self.SYSTEM  == WorkingMode.SIM:
            for i,drone in enumerate(self.drones_names):
                callback = lambda msg, idx=i: self.odom_callback(msg, idx)
                self.odom_subscribers.append(self.create_subscription(
                    odom_type[self.SYSTEM], drone + odom_name[self.SYSTEM], callback, 10))
            
        elif self.SYSTEM == WorkingMode.REAL:
            self.pose_sub = self.create_subscription(NamedPoseArray,"/poses", self.poses_callback,qos)

        else :
            raise Exception("Invalid working mode. Please choose either 'sim' or 'hardware' as backend parameter.")

        #start the main loops of the system with a timer method
        self.timer = self.create_timer(self.MANAGER_TIMER, self.mainloop) # type: ignore
            
        #set the starting state
        self.manager_state = ManagerState.WAITING_FOR_ODOMETRY
        self.agent_state   = None
        self.online_status = {i : 0 for i in range(len(self.AGENTS_INDICES))} # type: ignore
        self.get_logger().info(f"Working in {self.SYSTEM.name} mode. Manager state {self.manager_state.name} !!")
        self.min_dist = np.inf

# ──────── Subscription function ──────────────────────────────────────────────────────────────────

    def on_agent_message(self,msg):
        match (msg.data) :
            case AgentState.TAKEOFF:
                # Logic for ascending to target altitude
                self.agent_state = AgentState.TAKEOFF

            case AgentState.READY:
                # Logic for mission execution or hovering
                self.agent_state = AgentState.READY

            case AgentState.LANDING:
                # Logic for descending and disarming
                self.agent_state = AgentState.LANDING

            case AgentState.STOP:
                self.agent_state = AgentState.STOP

            case _:
                self.get_logger().info("Error: Unknown Agent State.")

    def odom_callback(self, msg, idx):
        if type(msg) == Odometry:
            self.pos[idx, 0] = msg.pose.pose.position.x
            self.pos[idx, 1] = msg.pose.pose.position.y
            self.pos[idx, 2] = msg.pose.pose.position.z

        elif type(msg) == PoseStamped:
            self.pos[idx, 0] = msg.pose.position.x
            self.pos[idx, 1] = msg.pose.position.y
            self.pos[idx, 2] = msg.pose.position.z

        self.online_status[idx] = 1
        self.compute_min_dist()

    def poses_callback(self, msg):
        for p in msg.poses:

            idx      = self.name_to_index.get(p.name)
            if idx is None:
                continue
            position = p.pose.position

            self.pos[idx, 0] = position.x
            self.pos[idx, 1] = position.y
            self.pos[idx, 2] = position.z

            self.online_status[idx] = 1
        self.compute_min_dist()

# ──────── Goals managment ────────────────────────────────────────────────────────────────────────

    def update_and_publish_goals(self, current_time) :
        # Check if it is time for the recalulcation     
        if len(self.recalc_times) == 0:
            self.get_logger().info(f"{AnsiColor.BOLD_GREEN} All tasks concluded. Sending landing command {AnsiColor.RESET}",throttle_duration_sec=5.0)
            self.landing_command_pub.publish(Bool(data=True))
            return

        if current_time > self.recalc_times[0]:
            self.recalc_times.pop(0) # empty list as tasks are finished
            if len(self.recalc_times): # when we reach the last recalculation time then the tasks are finnished
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Recaculating task at recalculation time : {current_time}" \
                                    f".Remaining recalculation times: {self.recalc_times} {AnsiColor.RESET}")
                self.compute_goals()

    def cost_func(self, x, moving_bots) :
        cost = 0
        for idx, bot in enumerate(moving_bots) :
            goal_pos = np.array([x[2*idx], x[2*idx+1]]) if self.DIM == 2 else np.array([x[3*idx], x[3*idx+2], x[3*idx+2]])
            cur_pos = self.pos[bot][:self.DIM]
            cost = max(cost, np.linalg.norm(cur_pos-goal_pos))
        return cost

    def compute_goals(self) :
        known_pos = {}
        fixed_goals = False
        waiting_task = set()
        moving_bots = []
        if len(self.recalc_times) > 0 :
            for _ in range(len(self.tasks)) :
                task = self.tasks[-1]
                if task.timespan[1] > self.recalc_times[0] :
                    break
                if task.is_goal :
                    # self.get_logger().info(f"{AnsiColor.VIOLET} new task : period : {task.timespan}, bot : {task.bot}, goal : {task.goal} {AnsiColor.RESET}")
                    fixed_goals = True
                    known_pos[task.bot] = task.goal
                    self.tasks.pop()
                elif fixed_goals :
                    edge = [self.AGENTS_INDICES[e] for e in task.edges] # type: ignore
                    # self.get_logger().info(f"{AnsiColor.VIOLET} new task : period : {task.timespan}, edge : {[self.AGENTS_INDICES[e] for e in task.edges]}, rel_pos : {task.rel_position} {AnsiColor.RESET}") # type: ignore
                    if edge[0] in known_pos.keys() and edge[1] in known_pos.keys():
                        if not np.array_equal(known_pos[edge[1]] - known_pos[edge[0]], np.array(task.rel_position)) :
                            self.get_logger().info(f"{AnsiColor.YELLOW} known_pos : {known_pos} {AnsiColor.RESET}")
                            self.get_logger().error(f"{AnsiColor.RED} error : the tasks are not feasible {AnsiColor.RESET}")
                            self.landing_command_pub.publish(Bool(data=True))
                            return
                    elif edge[0] in known_pos.keys() :
                        known_pos[edge[1]] = known_pos[edge[0]] + np.array(task.rel_position)
                        moving_bots.append(edge[1])
                    elif edge[1] in known_pos.keys() :
                        known_pos[edge[0]] = known_pos[edge[1]] - np.array(task.rel_position)
                        moving_bots.append(edge[0])
                    else :
                        waiting_task.add(task)
                    self.tasks.pop()
                else :
                    waiting_task.add(task)
                    self.tasks.pop()

            while len(waiting_task) > 0 :
                wait_copy = set()
                for task in waiting_task :
                    edge = [self.AGENTS_INDICES[e] for e in task.edges] # type: ignore
                    if len(known_pos) == 0 :
                        # self.get_logger().info(f"{AnsiColor.VIOLET} new task : period : {task.timespan}, edge : {[self.AGENTS_INDICES[e] for e in task.edges]}, rel_pos : {task.rel_position} {AnsiColor.RESET}") # type: ignore
                        known_pos[edge[0]] = np.array((0, 0)) if self.DIM == 2 else np.array((0, 0, 0))
                        known_pos[edge[1]] = np.array(task.rel_position)
                        moving_bots.append(edge[0])
                        moving_bots.append(edge[1])
                    elif edge[0] in known_pos.keys() and edge[1] in known_pos.keys():
                        if not np.array_equal(known_pos[edge[1]] - known_pos[edge[0]], np.array(task.rel_position)) :
                            self.get_logger().info(f"{AnsiColor.YELLOW} known_pos : {known_pos} {AnsiColor.RESET}")
                            self.get_logger().error(f"{AnsiColor.RED} error : the tasks are not feasible {AnsiColor.RESET}")
                            self.landing_command_pub.publish(Bool(data=True))
                            return
                    elif edge[0] in known_pos.keys() :
                        # self.get_logger().info(f"{AnsiColor.VIOLET} new task : period : {task.timespan}, edge : {[self.AGENTS_INDICES[e] for e in task.edges]}, rel_pos : {task.rel_position} {AnsiColor.RESET}") # type: ignore
                        known_pos[edge[1]] = known_pos[edge[0]] + np.array(task.rel_position)
                        moving_bots.append(edge[1])
                    elif edge[1] in known_pos.keys() :
                        # self.get_logger().info(f"{AnsiColor.VIOLET} new task : period : {task.timespan}, edge : {[self.AGENTS_INDICES[e] for e in task.edges]}, rel_pos : {task.rel_position} {AnsiColor.RESET}") # type: ignore
                        known_pos[edge[0]] = known_pos[edge[1]] - np.array(task.rel_position)
                        moving_bots.append(edge[0])
                    else :
                        wait_copy.add(task)
                waiting_task = wait_copy

            moving_bots.sort()

            if fixed_goals == False :
                # # Set the centre of the shape in (0, 0)
                min_x = min_y = 0
                max_x = max_y = 0
                max_z = min_z = 0
                for p in known_pos.values() :
                    min_x = min(min_x, p[0])
                    max_x = max(max_x, p[0])
                    min_y = min(min_y, p[1])
                    max_y = max(max_y, p[1])
                    if self.DIM == 3 :
                        min_z = min(min_z, p[2])
                        max_z = max(max_z, p[2])

                c = np.array((max_x + min_x, max_y + min_y))/2 if self.DIM == 2 else np.array((max_x + min_x, max_y + min_y, 2*(min_z - .5)))/2

                for k, v in known_pos.items() :
                    known_pos[k] = v - c

                # # # Place the drones by solving a minimization pb
                # TODO : to be updated for goal missions
                # constraints = []
                # x0 = [known_pos[moving_bots[0]][0], known_pos[moving_bots[0]][1]] if self.DIM == 2 else [known_pos[moving_bots[0]][0], known_pos[moving_bots[0]][1], known_pos[moving_bots[0]][2]]
                # for i in range(1, len(moving_bots)) :
                #     x0 += [known_pos[moving_bots[i]][0], known_pos[moving_bots[i]][1]] if self.DIM == 2 else [known_pos[moving_bots[i]][0], known_pos[moving_bots[i]][1], known_pos[moving_bots[i]][2]]
                #     A = np.zeros((self.DIM, self.DIM*len(moving_bots))) # type: ignore
                #     for d in range(self.DIM) : # type: ignore
                #         A[d, d] = 1
                #         A[d, i*self.DIM + d] = -1 # type: ignore
                #     dp = known_pos[moving_bots[0]] - known_pos[moving_bots[i]]
                #     constraints.append(LinearConstraint(A, dp, dp))
                # if self.DIM == 3 :
                #     A = np.zeros((len(moving_bots), self.DIM*len(moving_bots)))
                #     for i in range(len(moving_bots)) :
                #         A[i][3*i+2] = 1
                #     constraints.append(LinearConstraint(A, lb = 0.2))
                # res = minimize(lambda x : self.cost_func(x, moving_bots), x0, method="trust-constr", constraints=constraints)
                # self.get_logger().info(f"{AnsiColor.VIOLET} constraint violation : {res.constr_violation} {AnsiColor.RESET}")
                # # self.get_logger().info(f"{AnsiColor.VIOLET} result : {res.x} {AnsiColor.RESET}")
                # sol = res.x
                # for idx, bot in enumerate(moving_bots) :
                #     known_pos[bot] = np.array([sol[self.DIM*idx + d] for d in range(self.DIM)]) # type: ignore

        # self.get_logger().info(f"{AnsiColor.VIOLET} known_pos : {known_pos} {AnsiColor.RESET}")
        msg = Goallist()
        for i in self.AGENTS_INDICES : # type: ignore
            goal = Goal()
            goal.index = i
            point = Point()
            if i in known_pos.keys() :
                goal.has_one = True
                point.x = float(known_pos[i][0])
                point.y = float(known_pos[i][1])
                point.z = 0. if self.DIM == 2 else float(known_pos[i][2])
            else :
                goal.has_one = False
                point.x = 0.
                point.y = 0.
                point.z = 0.
            goal.pos = point
            msg.goals.append(goal)
            self.get_logger().info(f"{AnsiColor.VIOLET} sending goal : has_one : {goal.has_one}, index : {goal.index}, pos : {goal.pos.x, goal.pos.y, goal.pos.z} {AnsiColor.RESET}")
        self.goal_pub.publish(msg)

# ──────── Mainloop ───────────────────────────────────────────────────────────────────────────────

    def mainloop(self):
        if self.manager_state == ManagerState.WAITING_FOR_ODOMETRY:
            #check whether all drones have published their odometry
            if not 0 in self.online_status.values():
                #change state
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN}Received odom message from all agents, changing manager state to: {ManagerState.READY.name} {AnsiColor.RESET}")
                self.manager_state = ManagerState.READY

            else:
                missing_agents =  [key for key, value in self.online_status.items() if not value]
                missing_agents_id = [self.AGENTS_INDICES[i] for i in missing_agents] # type: ignore
                self.get_logger().info(f"Waiting for odom messages from agents: {missing_agents_id}. Missing agents: {missing_agents}",throttle_duration_sec=5.0)

        else :
            if self.agent_state == AgentState.READY:
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} RVO currently mode {self.agent_state.name}. {AnsiColor.RESET}",throttle_duration_sec=5.0)
        
                if self.start_time is None: # time is considered since the start of the mission
                    self.start_time = self.get_clock().now()
                
                current_time = self.get_clock().now() - self.start_time
                current_time = current_time.nanoseconds / 1e9
                
                self.update_and_publish_goals(current_time)

            elif self.agent_state == AgentState.STOP :
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} RVO externally stopped . {AnsiColor.RESET}",throttle_duration_sec=5.0)

            elif self.agent_state == AgentState.LANDING :
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} RVO landing mode . {AnsiColor.RESET}",throttle_duration_sec=5.0)
                self.get_logger().info(f"{AnsiColor.BOLD_GREEN} Min distance between two drone : {self.min_dist} {AnsiColor.RESET}",throttle_duration_sec=5.0)

# ──────── Utils ──────────────────────────────────────────────────────────────────────────────────

    def _load_tasks(self):
        """
        Load TASKS from ROS 2 parameters using prefix parsing.
        taken from https://gist.github.com/agrueneberg/d76fff493753fa531f1b5d33be0f07ed
        """

        params_task = self.get_parameters_by_prefix('TASKS')    # e.g., {"task_3.timespan":[1,2],"task_3.rel_position":[1,2], "task_2.rel_position"...}
        params_period = self.get_parameters_by_prefix('PERIODS')# e.g., {"period_1":[1,2],"period_2":[1,2], ...}

        if not params_task:
            self.get_logger().warn("No TASKS parameters found")
            return []

        tasks_by_name = {}

        for full_key, param in params_task.items():
            # Example: "task_3.timespan"
            task_name, field = full_key.split(".", 1)

            tasks_by_name.setdefault(task_name, {})[field] = param.value # type: ignore

            self.get_logger().info(
                f"Loaded param: {task_name}.{field} = {param.value}" # type: ignore
            )
        
        periods = []
        
        for full_key, param in params_period.items():
            # Example: "period_1" -> period are assumed to be given in order
            _, num = full_key.split("_", 1)
            num = int(num)
            periods.append(param.value) # type: ignore

            self.get_logger().info(
                f"Loaded period: {num} = {param.value}" # type: ignore
            )

        tasks = []
        for name, data in tasks_by_name.items():
            # remap edges to to the agent indices. Agents in the algorithm go from 1 to n
            # but user might decide different indices in the hardware setup 
            if 'goal' in data :
                tasks.append(Task(
                    timespan = periods[data["period_num"]],
                    is_goal = True,
                    bot = data["bot"],
                    period_num= data["period_num"],
                    goal = data["goal"]
                ))
            else :
                try :
                    data['edges'] = (self.AGENTS_INDICES.index(data['edges'][0]), self.AGENTS_INDICES.index(data['edges'][1])) # type: ignore
                except ValueError as e:
                    raise ValueError(f"Task {name} has edges {data['edges']} which include agent indices not in AGENTS_INDICES parameter {self.AGENTS_INDICES}. Please make sure all agents involved in tasks are included in AGENTS_INDICES.") from e

                try:
                    tasks.append(Task(
                                    timespan    = periods[data['period_num']],
                                    is_goal= False,
                                    edges       = data['edges'],
                                    rel_position= data['rel_position'],
                                    size        = data['size'],
                                    period_num  = data['period_num'],
                                    operator    = data.get('operator', 'always')
                                    )
                                )
                except Exception as e:
                    raise e
            
        tasks.sort(key = lambda x : x.timespan[1], reverse= True)

        # check phase : make sure periods are ordered and not overlapping
        for num, period in enumerate(periods):
            for other_num, other_period in enumerate(periods):
                if num < other_num:
                    if not (period[1] <= other_period[0]):
                        self.get_logger().error(
                            f"Periods {num} and {other_num} are overlapping or are unordred. PLease make sure the period are orderd in ascending order: {period} and {other_period}. Please provide "
                        )
        self.get_logger().info('\033[32m'+ f" All tasks loaded successfully !" + '\033[0m')
        return tasks, periods

    def recalculate_at(self):
        """
        Trigger a recalculation every time ther task ends
        """
        timespans = [task.timespan[1] for task in self.tasks] + [0] # every time a task end we can recalculate the tasks. Added zero for initial calculation
        timespans = list(set(timespans))                              # remove duplicates and retrun ordered list
        timespans.sort()
        return timespans

    def compute_min_dist(self) :
        for i in range(self.NUM_AGENTS) :
            if self.online_status[i] :
                for j in range(i+1, self.NUM_AGENTS) :
                    if self.online_status[j] :
                        self.min_dist = min(self.min_dist, np.linalg.norm(self.pos[i]-self.pos[j]))


def signal_handler(sig, frame):
    print("Shutdown signal received, cleaning up...")
    if (rclpy.utilities.ok()) : # type: ignore
        rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    manager = Manager()

    rclpy.spin(manager)
    manager.destroy_node()
    if rclpy.utilities.ok() : # type: ignore
        rclpy.shutdown()

if __name__ == "__main__":
    main()
