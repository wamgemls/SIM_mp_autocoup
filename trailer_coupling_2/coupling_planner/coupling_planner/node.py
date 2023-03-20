import traceback
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, TransformStamped, Buffer, LookupException, Time, Duration, BufferInterface
from squaternion import Quaternion
import numpy as np

from pln_interfaces.srv import CouplingPlannerMode
from pln_interfaces.msg import ControllerTrajectory, ControllerTrajectoryPoint
from pln_interfaces.msg import Ego,TwistSingle,Vector3Single,AutoboxVehicle,CouplingPlannerData
from geometry_msgs.msg import PoseStamped

from .planner import CouplingPlanner, PlannerMode, Pose, TrajectoryPoint
from .tf_tools import do_transform_pose_stamped, invert_transform,combine_transform

class CouplingPlannerNode(Node):
 
    def __init__(self):
        super().__init__('pln_coupling_planner')
        
        #node parameter
        self.declare_parameter("timer_period_seconds", 0.2)

        self.declare_parameter("planner_frame", "truck_odom")
        self.declare_parameter("path_res", 0.1)
        self.declare_parameter("path23_res", 0.1)
        self.declare_parameter("vx", -0.55)
        self.declare_parameter("acc_time", 0.5)
        self.declare_parameter("dec_time", 0.5)
        self.declare_parameter("history_point_limit", 3)
        self.declare_parameter("trajectory_backup", 0.0)
        self.declare_parameter("ego_delta_bilevel", 0.5)
        self.declare_parameter("goal_delta_bilevel", 0.5)
        self.declare_parameter("max_curvature", 0.3)
        self.declare_parameter("min_traj_length", 2.0)
        self.declare_parameter("max_traj_length", 50.0)
        self.declare_parameter("dis_prekingpin_kingpin", 0.525)

        #services
        self.planner_mode_srv = self.create_service(CouplingPlannerMode,"/coupling_planner/set_mode",self.planner_mode_service_callback)

        #subscriber
        self.ego_subscription = self.create_subscription(Ego,"/ego_input",self.ego_subscriber_callback,10)
        self.kingpin_pose_subscription = self.create_subscription(PoseStamped,"/pln/kingpin_planner",self.kingpin_pose_subscriber_callback,10)

        #publisher
        self.controller_trajectory_publisher = self.create_publisher(ControllerTrajectory, "/pln/gpu/trajectory",10)
        self.coupling_planner_data_publisher = self.create_publisher(CouplingPlannerData,"/pln/coupling_planner_data",10)

        #timer
        self.timer_init = self.create_timer(2.5,self.init)

        #transformation
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer,self)

        #planner
        self.planner = CouplingPlanner( path_res=self.get_parameter("path_res").get_parameter_value().double_value, 
                                        path23_res=self.get_parameter("path23_res").get_parameter_value().double_value, 
                                        vx=self.get_parameter("vx").get_parameter_value().double_value, 
                                        acc_time=self.get_parameter("acc_time").get_parameter_value().double_value,
                                        dec_time=self.get_parameter("dec_time").get_parameter_value().double_value,
                                        history_point_limit=self.get_parameter("history_point_limit").get_parameter_value().integer_value, 
                                        trajectory_backup=self.get_parameter("trajectory_backup").get_parameter_value().double_value,
                                        ego_delta_bilevel=self.get_parameter("ego_delta_bilevel").get_parameter_value().double_value, 
                                        goal_delta_bilevel=self.get_parameter("goal_delta_bilevel").get_parameter_value().double_value, 
                                        max_curvature=self.get_parameter("max_curvature").get_parameter_value().double_value, 
                                        min_traj_length=self.get_parameter("min_traj_length").get_parameter_value().double_value,
                                        max_traj_length=self.get_parameter("max_traj_length").get_parameter_value().double_value,
                                        dis_prekingpin_kingpin=self.get_parameter("dis_prekingpin_kingpin").get_parameter_value().double_value
                                        )
        
        self.kingpin_truckbase_posestamped = PoseStamped()

        self.tf_truckodom2truckbase = TransformStamped()
        self.tf_truckbase2kingpinbase = TransformStamped()

        self.ego_msg = Ego()

    def init(self):
        self.timer_init.cancel()

        if self.get_parameter("planner_frame").get_parameter_value().string_value == "truck_odom":
            self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle_truckodom)
        elif self.get_parameter("planner_frame").get_parameter_value().string_value == "kingpin_base":
            self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle_kingpinbase)
        else:
            self.get_logger().info("ERROR - invalid planner_frame")

    def cycle_truckodom(self):
        self.get_logger().info("Cycle - truck_odom - Mode: {}".format(str(self.planner.planner_mode)))
        
        self.tf_read()
        self.pose2planner_truckodom()
        self.planner.prekingpin()

        self.publish_controller_trajectory(combine_transform(self.tf_truckodom2truckbase,invert_transform(self.tf_truckodom2truckbase)))
        self.publish_coupling_planner_data()

    def cycle_kingpinbase(self):
        self.get_logger().info("Cycle - kingpin_base - Mode: {}".format(str(self.planner.planner_mode)))
        
        self.tf_read()
        self.pose2planner_kingpinbase()
        self.planner.prekingpin()

        self.publish_controller_trajectory(combine_transform(invert_transform(self.tf_truckbase2kingpinbase),invert_transform(self.tf_truckodom2truckbase)))
        self.publish_coupling_planner_data()

    def tf_read(self):
        
        try:
            self.tf_truckodom2truckbase = self.transform_buffer.lookup_transform("truck_base", "truck_odom", Time())
        except LookupException:
            self.get_logger().info("ERROR - no tf data - truckodom2truckbase")

        try:
            kingpin2truckbase = TransformStamped()
            kingpin2truckbase.transform.translation.x = self.kingpin_truckbase_posestamped.pose.position.x
            kingpin2truckbase.transform.translation.y = self.kingpin_truckbase_posestamped.pose.position.y
            kingpin2truckbase.transform.translation.z = self.kingpin_truckbase_posestamped.pose.position.z

            kingpin2truckbase.transform.rotation.w = self.kingpin_truckbase_posestamped.pose.orientation.w
            kingpin2truckbase.transform.rotation.x = self.kingpin_truckbase_posestamped.pose.orientation.x
            kingpin2truckbase.transform.rotation.y = self.kingpin_truckbase_posestamped.pose.orientation.y
            kingpin2truckbase.transform.rotation.z = self.kingpin_truckbase_posestamped.pose.orientation.z

            self.tf_truckbase2kingpinbase = invert_transform(kingpin2truckbase)
        except:
            self.get_logger().info("ERROR - no kingpin data - truckbase2kingpin")


    def pose2planner_truckodom(self):

        #convert PoseStamped ego into PlannerPose
        tf_truckbase2truckodom = invert_transform(self.tf_truckodom2truckbase)

        ego_pose = Pose()
        ego_pose.x = tf_truckbase2truckodom.transform.translation.x
        ego_pose.y = tf_truckbase2truckodom.transform.translation.y
        q = Quaternion( tf_truckbase2truckodom.transform.rotation.w,
                        tf_truckbase2truckodom.transform.rotation.x,
                        tf_truckbase2truckodom.transform.rotation.y,
                        tf_truckbase2truckodom.transform.rotation.z
                        )
        roll, pitch, yaw = q.to_euler()
        ego_pose.yaw = yaw

        self.get_logger().info("ego@truckodom: x: {}, y: {}, yaw: {}".format(ego_pose.x,ego_pose.y,ego_pose.yaw))
        
        if -0.2 < self.ego_msg.velocity.linear.x < 0.2:
            ego_pose.vx = 0.0
            ego_pose.curvature = 0.0
        else:
            ego_pose.vx = self.ego_msg.velocity.linear.x
            ego_pose.curvature = self.planner.divide(self.ego_msg.velocity.angular.z,self.ego_msg.velocity.linear.x)

        #convert PoseStamped goal into PlannerPose
        goal_posestamped = do_transform_pose_stamped(self.kingpin_truckbase_posestamped,invert_transform(self.tf_truckodom2truckbase))

        goal_pose = Pose()
        goal_pose.x = goal_posestamped.pose.position.x
        goal_pose.y = goal_posestamped.pose.position.y
        q = Quaternion( goal_posestamped.pose.orientation.w,
                        goal_posestamped.pose.orientation.x,
                        goal_posestamped.pose.orientation.y,
                        goal_posestamped.pose.orientation.z
                        )
        roll, pitch, yaw = q.to_euler()
        goal_pose.yaw = yaw

        goal_pose.vx = 0.0
        goal_pose.curvature = 0.0

        self.get_logger().info("goal@truckodom: x: {}, y: {}, yaw: {}".format(goal_pose.x,goal_pose.y,goal_pose.yaw))
        
        #update planner pose
        if abs(ego_pose.x) < 10000 and abs(goal_pose.x) < 10000 and\
            abs(ego_pose.y) < 10000 and abs(goal_pose.y) < 10000: 
            self.planner.update_pose(ego_pose,goal_pose)
        else:
            self.get_logger().info("ERROR - invalid pose")

    def pose2planner_kingpinbase(self):
    
        #convert PoseStamped ego into PlannerPose
        ego_pose = Pose()
        ego_pose.x = self.tf_truckbase2kingpinbase.transform.translation.x
        ego_pose.y = self.tf_truckbase2kingpinbase.transform.translation.y
        q = Quaternion( self.tf_truckbase2kingpinbase.transform.rotation.w,
                        self.tf_truckbase2kingpinbase.transform.rotation.x,
                        self.tf_truckbase2kingpinbase.transform.rotation.y,
                        self.tf_truckbase2kingpinbase.transform.rotation.z
                        )
        roll, pitch, yaw = q.to_euler()
        ego_pose.yaw = yaw

        self.get_logger().info("ego@kingpinbase: x: {}, y: {}, yaw: {}".format(ego_pose.x,ego_pose.y,ego_pose.yaw))
        
        if -0.2 < self.ego_msg.velocity.linear.x < 0.2:
            ego_pose.vx = 0.0
            ego_pose.curvature = 0.0
        else:
            ego_pose.vx = self.ego_msg.velocity.linear.x
            ego_pose.curvature = self.planner.divide(self.ego_msg.velocity.angular.z,self.ego_msg.velocity.linear.x)

        #convert PoseStamped goal into PlannerPose
        goal_posestamped = do_transform_pose_stamped(self.kingpin_truckbase_posestamped,self.tf_truckbase2kingpinbase)

        goal_pose = Pose()
        goal_pose.x = goal_posestamped.pose.position.x
        goal_pose.y = goal_posestamped.pose.position.y
        q = Quaternion( goal_posestamped.pose.orientation.w,
                        goal_posestamped.pose.orientation.x,
                        goal_posestamped.pose.orientation.y,
                        goal_posestamped.pose.orientation.z
                        )
        roll, pitch, yaw = q.to_euler()
        goal_pose.yaw = yaw

        self.get_logger().info("goal@kingpinbase: x: {}, y: {}, yaw: {}".format(goal_pose.x,goal_pose.y,goal_pose.yaw))

        goal_pose.vx = 0.0
        goal_pose.curvature = 0.0
        
        #update planner pose
        if abs(ego_pose.x) < 10000 and abs(goal_pose.x) < 10000 and\
            abs(ego_pose.y) < 10000 and abs(goal_pose.y) < 10000: 
            self.planner.update_pose(ego_pose,goal_pose)
        else:
            self.get_logger().info("ERROR - invalid pose")
        
    def planner_mode_service_callback(self, request, response):

        if request.planner_mode is request.COUPLING_PHASE_STANDSTILL:
            self.planner.planner_mode = PlannerMode.STANDSTILL
            response.success = True
        elif request.planner_mode is request.COUPLING_PHASE_TILL_PREKINGPIN:
            self.planner.planner_mode = PlannerMode.COUPLING_PHASE_PREKINGPIN
            response.success = True
        elif request.planner_mode is request.COUPLING_PHASE_TILL_KINGPIN:
            self.planner.planner_mode = PlannerMode.COUPLING_PHASE_KINGPIN
            response.success = True
        else:
            self.get_logger().info("TM - Modeswitch: fail")
            response.success = False

        self.get_logger().info("TM - Modeswitch: {}".format(str(self.planner.planner_mode)))
        return response

    def ego_subscriber_callback(self,ego_input):
        self.ego_msg = ego_input
    
    def kingpin_pose_subscriber_callback(self, kingpin_pose_msg):
        self.kingpin_truckbase_posestamped = kingpin_pose_msg
        
    def publish_controller_trajectory(self,transform):

        msg = ControllerTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "truck_odom"
        msg.control_policy = 10
        msg.coordinate_system = 10
        msg.indicator_bitmask = 0

        for i in range(len(self.planner.trajectory23)):

            pose_stamped_plannerframe = self.conversion_pose2posestamped(Pose(  x=self.planner.trajectory23[i].x,
                                                                                y=self.planner.trajectory23[i].y,
                                                                                yaw=self.planner.trajectory23[i].yaw
                                                                                ))

            pose_stamped_truckodom = do_transform_pose_stamped(pose_stamped_plannerframe,transform)

            msg.points[i].t = self.planner.trajectory23[i].t
            msg.points[i].s = self.planner.trajectory23[i].s
            msg.points[i].x = pose_stamped_truckodom.pose.position.x
            msg.points[i].y = pose_stamped_truckodom.pose.position.y
            msg.points[i].vx = self.planner.trajectory23[i].vx
            msg.points[i].ax = self.planner.trajectory23[i].ax

            q = Quaternion( pose_stamped_truckodom.pose.orientation.w,
                            pose_stamped_truckodom.pose.orientation.x,
                            pose_stamped_truckodom.pose.orientation.y,
                            pose_stamped_truckodom.pose.orientation.z
                            )
            roll, pitch, yaw = q.to_euler()

            msg.points[i].yaw = yaw
            msg.points[i].curvature = self.planner.trajectory23[i].curvature
            msg.points[i].hitch_angle = 0.0

            if self.get_parameter("vx").get_parameter_value().double_value < 0.0:
                msg.points[i].gear_state = 30
            else:
                msg.points[i].gear_state = 10

        self.controller_trajectory_publisher.publish(msg)


    def publish_coupling_planner_data(self):

        msg = CouplingPlannerData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "-"

        msg.ego_pose = self.conversion_pose2posestamped(self.planner.ego_pose)
        msg.kingpin_goal_pose = self.conversion_pose2posestamped(self.planner.kingpin_goal_pose)
        msg.prekingpin_goal_pose = self.conversion_pose2posestamped(self.planner.prekingpin_goal_pose)

        msg.trajectory = self.conversion_trajectory2controllertrajectory(self.planner.trajectory)
        msg.trajectory23 = self.conversion_trajectory2controllertrajectory(self.planner.trajectory23)

        msg.ego_vx = self.planner.ego_pose.vx
        msg.ego_curvature = self.planner.ego_pose.curvature

        self.coupling_planner_data_publisher.publish(msg)

    def conversion_trajectory2controllertrajectory(self,trajectory):

        controllertrajectorylist = []

        if trajectory:
            for trajectorypoint in trajectory:
                new_controllertrajectorypoint = ControllerTrajectoryPoint()
                new_controllertrajectorypoint.t = float(trajectorypoint.t)
                new_controllertrajectorypoint.s = float(trajectorypoint.s)
                new_controllertrajectorypoint.x = float(trajectorypoint.x)
                new_controllertrajectorypoint.y = float(trajectorypoint.y)
                new_controllertrajectorypoint.yaw = float(trajectorypoint.yaw)
                new_controllertrajectorypoint.vx = float(trajectorypoint.vx)
                new_controllertrajectorypoint.ax = float(trajectorypoint.ax)
                new_controllertrajectorypoint.curvature = float(trajectorypoint.curvature)
                new_controllertrajectorypoint.hitch_angle = 0.0
                new_controllertrajectorypoint.gear_state = 30
                controllertrajectorylist.append(new_controllertrajectorypoint)

        return controllertrajectorylist

    def conversion_pose2posestamped(self,pose):

        posestamped = PoseStamped()
        posestamped.header.stamp = self.get_clock().now().to_msg()
        posestamped.header.frame_id = "-"
        
        posestamped.pose.position.x = float(pose.x)
        posestamped.pose.position.y = float(pose.y)
        posestamped.pose.position.z = 0.0
        
        q = Quaternion.from_euler(0.0,0.0,float(pose.yaw))
        posestamped.pose.orientation.w = float(q.w)
        posestamped.pose.orientation.x = float(q.x)
        posestamped.pose.orientation.y = float(q.y)
        posestamped.pose.orientation.z = float(q.z)

        return posestamped


def main(args=None):
    rclpy.init(args=args)
    coupling_planner_node = CouplingPlannerNode()
    rclpy.spin(coupling_planner_node)
    coupling_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
