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

from .tools import CouplingPlanner, PlannerMode, Pose, TrajectoryPoint
from kingpin_broadcaster.tools import do_transform_pose_stamped

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
        self.ego_subscription = self.create_subscription(Ego,"/gw/odom/truck",self.ego_subscriber_callback,10)
        self.kingpin_pose_subscription = self.create_subscription(PoseStamped,"/pln/kingpin_emulation",self.kingpin_pose_subscriber_callback,10)

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

        self.tf_kingpin2goal = TransformStamped()
        self.tf_kingpin2goal.transform.translation.x = -0.529
        self.tf_kingpin2goal.transform.translation.y = 0.0
        self.tf_kingpin2goal.transform.translation.z = 0.0
        self.tf_kingpin2goal.transform.rotation.w = 1.0
        self.tf_kingpin2goal.transform.rotation.x = 0.0
        self.tf_kingpin2goal.transform.rotation.y = 0.0
        self.tf_kingpin2goal.transform.rotation.z = 0.0

        self.tf_truckbase2truckodom = TransformStamped()

        self.ego_pose = Pose()
        self.goal_pose = Pose()

        self.ego_pose_truckodom = PoseStamped()
        self.kingpin_pose_truckodom = PoseStamped()
        self.kingpin_pose_truckbase = PoseStamped()
        self.goal_pose_truckodom = PoseStamped()

        self.ego_pose_kingpinbase = PoseStamped()
        self.kingpin_pose_kingpinbase = PoseStamped()
        self.goal_pose_kingpinbase = PoseStamped()

        self.ego_msg = Ego()

        self.planner.planner_mode = PlannerMode.SIMULATION
        

    def init(self):
        self.timer_init.cancel()

        if self.get_parameter("planner_frame").get_parameter_value().string_value == "truck_odom":
            self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle_truckodom)
        elif self.get_parameter("planner_frame").get_parameter_value().string_value == "kingpin_base":
            self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle_kingpinbase)
        else:
            self.get_logger().info("ABORT - invalid planner_frame")

    def cycle_truckodom(self):
        self.get_logger().info("Cycle - truck_odom - {}".format(str(self.planner.planner_mode)))

        self.pose2planner_truckodom()
        self.planner.cycle()
        self.publish_controller_trajectory_truckodom()
        self.publish_coupling_planner_data()

    def cycle_kingpinbase(self):
        self.get_logger().info("Cycle - kingpin_base - {}".format(str(self.planner.planner_mode)))
        
        self.pose2planner_kingpinbase()
        self.planner.cycle()
        self.publish_controller_trajectory_kingpinbase()
        self.publish_coupling_planner_data()

    def pose2planner_truckodom(self):
 
        self.give_ego_in_truckodom()
        #self.kingpin_pose_truckodom = self.transform_truckbase2truckodom(self.kingpin_pose_truckbase)

        #validate age of ego_pose, kingpin_pose, ego_vx
        now = self.get_clock().now().nanoseconds/1e9
        age_ego_tf = now - self.ego_pose_truckodom.header.stamp.nanosec/1e9
        age_kingpin = now - self.kingpin_pose_truckodom.header.stamp.nanosec/1e9
        age_ego_msg = now - self.ego_msg.header.stamp.nanosec/1e9

        if 0 < age_ego_tf <= 1 and 0 < age_kingpin <= 1 and 0 < age_ego_msg <= 1:

            #convert PoseStamped Ego into PlannerPose
            self.ego_pose.x = self.ego_pose_truckodom.pose.position.x
            self.ego_pose.y = self.ego_pose_truckodom.pose.position.y
            q = Quaternion( self.ego_pose_truckodom.pose.orientation.w,
                            self.ego_pose_truckodom.pose.orientation.x,
                            self.ego_pose_truckodom.pose.orientation.y,
                            self.ego_pose_truckodom.pose.orientation.z
                            )
            roll, pitch, yaw = q.to_euler()
            self.ego_pose.yaw = yaw
            
            self.ego_pose.vx = 0.0#self.ego_msg.velocity.linear.x
            self.ego_pose.curvature = 0.0#self.planner.divide(self.ego_msg.velocity.angular.z/self.ego_msg.velocity.linear.x)

            #convert PoseStamped goal into PlannerPose

            self.goal_pose_truckodom = self.kingpin_pose_truckodom

            self.goal_pose.x = self.goal_pose_truckodom.pose.position.x
            self.goal_pose.y = self.goal_pose_truckodom.pose.position.y
            q = Quaternion( self.goal_pose_truckodom.pose.orientation.w,
                            self.goal_pose_truckodom.pose.orientation.x,
                            self.goal_pose_truckodom.pose.orientation.y,
                            self.goal_pose_truckodom.pose.orientation.z
                            )
            roll, pitch, yaw = q.to_euler()
            self.goal_pose.yaw = yaw
            
            self.goal_pose.vx = 0.0
            self.goal_pose.curvature = 0.0
            
            #update planner pose
            self.planner.update_pose(self.ego_pose,self.goal_pose)

        else:
            self.planner.planner_mode = PlannerMode.SIMULATION
            self.get_logger().info("TIMEOUT - invalid message input")

    def pose2planner_kingpinbase(self):
    
        self.give_ego_in_kingpinbase()

        #validate age of ego_pose, kingpin_pose, ego_vx
        now = self.get_clock().now().nanoseconds/1e9
        age_ego_tf = now - self.ego_pose_kingpinbase.header.stamp.nanosec/1e9
        age_ego_msg = now - self.ego_msg.header.stamp.nanosec/1e9

        if 0 < age_ego_tf <= 1 and 0 < age_ego_msg <= 1:

            #convert PoseStamped ego into PlannerPose
            self.ego_pose.x = self.ego_pose_kingpinbase.pose.position.x
            self.ego_pose.y = self.ego_pose_kingpinbase.pose.position.y
            q = Quaternion( self.ego_pose_kingpinbase.pose.orientation.w,
                            self.ego_pose_kingpinbase.pose.orientation.x,
                            self.ego_pose_kingpinbase.pose.orientation.y,
                            self.ego_pose_kingpinbase.pose.orientation.z
                            )
            roll, pitch, yaw = q.to_euler()
            self.ego_pose.yaw = yaw
            
            self.ego_pose.vx = self.ego_msg.velocity.linear.x
            self.ego_pose.curvature = self.planner.divide(self.ego_msg.velocity.angular.z/self.ego_msg.velocity.linear.x)

            #convert PoseStamped goal into PlannerPose

            self.goal_pose_kingpinbase = do_transform_pose_stamped(self.kingpin_pose_kingpinbase,self.tf_kingpin2goal)

            self.goal_pose.x = self.goal_pose_kingpinbase.pose.position.x
            self.goal_pose.y = self.goal_pose_kingpinbase.pose.position.y
            q = Quaternion( self.goal_pose_kingpinbase.pose.orientation.w,
                            self.goal_pose_kingpinbase.pose.orientation.x,
                            self.goal_pose_kingpinbase.pose.orientation.y,
                            self.goal_pose_kingpinbase.pose.orientation.z
                            )
            roll, pitch, yaw = q.to_euler()
            self.goal_pose.yaw = yaw

            self.goal_pose.vx = 0.0
            self.goal_pose.curvature = 0.0
            
            #update planner pose
            self.planner.update_pose(self.ego_pose,self.goal_pose)

        else:
            self.planner.planner_mode = PlannerMode.SIMULATION
            self.get_logger().info("TIMEOUT - invalid message input")

    def planner_mode_service_callback(self, request, response):

        if request.planner_mode is request.COUPLING_PHASE_STANDSTILL:
            self.planner.planner_mode = PlannerMode.STANDSTILL
        elif request.planner_mode is request.COUPLING_PHASE_TILL_PREKINGPIN:
            self.planner.planner_mode = PlannerMode.COUPLING_PHASE_PREKINGPIN
        elif request.planner_mode is request.COUPLING_PHASE_TILL_KINGPIN:
            self.planner.planner_mode = PlannerMode.COUPLING_PHASE_KINGPIN

        self.get_logger().info("modeswitch received: {}".format(str(self.planner.planner_mode)))

        response.success = bool(True)

    def ego_subscriber_callback(self,ego_msg):
        self.ego_msg = ego_msg
    
    def kingpin_pose_subscriber_callback(self, kingpin_pose_msg):
        #self.kingpin_pose_truckodom = self.transform_truckbase2truckodom(kingpin_pose_msg)
        self.kingpin_pose_truckodom= kingpin_pose_msg


    def give_ego_in_truckodom(self):
        try:
            transform = self.transform_buffer.lookup_transform("truck_odom", "truck_base", Time())
            self.get_logger().info("ego_in_truckodom: x: {}, y: {}".format( transform.transform.translation.x, 
                                                                            transform.transform.translation.y))

            self.tf_truckbase2truckodom = transform
            
            self.ego_pose_truckodom.header.stamp = self.get_clock().now().to_msg()
            self.ego_pose_truckodom.header.frame_id = "truck_odom"
            self.ego_pose_truckodom.pose.position.x = transform.transform.translation.x
            self.ego_pose_truckodom.pose.position.y = transform.transform.translation.y
            self.ego_pose_truckodom.pose.position.z = transform.transform.translation.z
            self.ego_pose_truckodom.pose.orientation.w = transform.transform.rotation.w
            self.ego_pose_truckodom.pose.orientation.x = transform.transform.rotation.x
            self.ego_pose_truckodom.pose.orientation.y = transform.transform.rotation.y
            self.ego_pose_truckodom.pose.orientation.z = transform.transform.rotation.z

        except LookupException:
            self.get_logger().info("no tf data - ego_in_truckodom")

    def give_ego_in_kingpinbase(self):
        try:
            transform = self.transform_buffer.lookup_transform("kingpin_base", "truck_base", Time())
            self.get_logger().info("ego_in_kingpinbase: x: {}, y: {}".format(   transform.transform.translation.x, 
                                                                                transform.transform.translation.y))
            
            self.ego_pose_kingpinbase.header.stamp = self.get_clock().now().to_msg()
            self.ego_pose_kingpinbase.header.frame_id = "kingpin_base"
            self.ego_pose_kingpinbase.pose.position.x = transform.transform.translation.x
            self.ego_pose_kingpinbase.pose.position.y = transform.transform.translation.y
            self.ego_pose_kingpinbase.pose.position.z = transform.transform.translation.z
            self.ego_pose_kingpinbase.pose.orientation.w = transform.transform.rotation.w
            self.ego_pose_kingpinbase.pose.orientation.x = transform.transform.rotation.x
            self.ego_pose_kingpinbase.pose.orientation.y = transform.transform.rotation.y
            self.ego_pose_kingpinbase.pose.orientation.z = transform.transform.rotation.z

        except LookupException:
            self.get_logger().info("no tf data - ego_in_kingpinbase")

    def transform_truckbase2truckodom(self, pose_truckbase):
        try:
            transform = self.transform_buffer.lookup_transform("truck_odom", "truck_base", Time())
            pose_truckodom = do_transform_pose_stamped(pose_truckbase,transform)
            self.get_logger().info("kingpin@odom: x: {}, y: {}".format( self.kingpin_pose_truckodom.pose.position.x, 
                                                                        self.kingpin_pose_truckodom.pose.position.y))
            return pose_truckodom
        except LookupException:
            self.get_logger().info(f"no tf data - truckbase2truckodom\n{traceback.format_exc()}")

    def publish_controller_trajectory_truckodom(self):

        msg = ControllerTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "truck_odom"
        msg.control_policy = 10
        msg.coordinate_system = 10
        msg.indicator_bitmask = 0

        if len(self.planner.trajectory23) == 23 and len(msg.points) == 23:
            for i in range(len(self.planner.trajectory23)):
                msg.points[i].t = self.planner.trajectory23[i].t
                msg.points[i].s = self.planner.trajectory23[i].s
                msg.points[i].x = self.planner.trajectory23[i].x
                msg.points[i].y = self.planner.trajectory23[i].y
                msg.points[i].vx = self.planner.trajectory23[i].vx
                msg.points[i].ax = self.planner.trajectory23[i].ax
                msg.points[i].yaw = self.planner.trajectory23[i].yaw
                msg.points[i].curvature = self.planner.trajectory23[i].curvature
                msg.points[i].hitch_angle = 0.0
                msg.points[i].gear_state = 30
            self.controller_trajectory_publisher.publish(msg)
        else:
            self.get_logger().info("wrong length of trajectory")

    def publish_controller_trajectory_kingpinbase(self):

        try:
            transform = self.transform_buffer.lookup_transform("truck_odom", "kingpin_base", Time())
            
            msg = ControllerTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "truck_odom"
            msg.control_policy = 10
            msg.coordinate_system = 10
            msg.indicator_bitmask = 0

            if len(self.planner.trajectory23) == 23 and len(msg.points) == 23:
                for i in range(len(self.planner.trajectory23)):

                    pose_stamped_kingpinbase = self.conversion_pose2posestamped(Pose(   x=self.planner.trajectory23[i].x,
                                                                                        y=self.planner.trajectory23[i].y,
                                                                                        yaw=self.planner.trajectory23[i].yaw
                                                                                        ))
                    pose_stamped_kingpinbase.header.frame_id = "kingpin_base"
                    pose_stamped_truckodom = do_transform_pose_stamped(pose_stamped_kingpinbase,transform)

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
                    msg.points[i].gear_state = 30

                self.controller_trajectory_publisher.publish(msg)
            else:
                self.get_logger().info("wrong length of trajectory")

        except LookupException:
            self.get_logger().info("no tf data - kingpinbase2truckodom")

    def publish_coupling_planner_data(self):

        msg = CouplingPlannerData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "-"

        msg.ego_pose_truckodom = self.ego_pose_truckodom
        msg.kingpin_pose_truckodom = self.kingpin_pose_truckodom
        msg.goal_pose_truckodom = self.goal_pose_truckodom

        msg.ego_pose_kingpinbase = self.ego_pose_kingpinbase
        msg.kingpin_pose_kingpinbase = self.ego_pose_kingpinbase
        msg.goal_pose_kingpinbase = self.goal_pose_kingpinbase

        msg.ego = self.conversion_pose2posestamped(self.planner.ego_pose)
        msg.prekingpin = self.conversion_pose2posestamped(self.planner.prekingpin_pose)
        msg.kingpin = self.conversion_pose2posestamped(self.planner.kingpin_pose)
        msg.trajectory_prekingpin = self.conversion_trajectory2controllertrajectory(self.planner.trajectory_p1)
        msg.trajectory_kingpin = self.conversion_trajectory2controllertrajectory(self.planner.trajectory_p2)
        msg.trajectory23 = self.conversion_trajectory2controllertrajectory(self.planner.trajectory23)

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
