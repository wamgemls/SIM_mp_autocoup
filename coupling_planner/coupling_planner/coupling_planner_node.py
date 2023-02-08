import numpy as np
import time

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, TransformStamped, Buffer, LookupException, Time, Duration, BufferInterface
from squaternion import Quaternion

from geometry_msgs.msg import PoseStamped

from pln_interfaces.srv import CouplingPlannerMode

from pln_interfaces.msg import ControllerTrajectory, ControllerTrajectoryPoint
from pln_interfaces.msg import Ego,TwistSingle,Vector3Single,AutoboxVehicle

from .coupling_planner_tools import CouplingPlanner, PlannerMode, Pose, TrajectoryPoint

from kingpin_emulation.kingpin_emulation_tools import do_transform_pose_stamped

class CouplingPlannerNode(Node):
 
    def __init__(self):
        super().__init__('pln_coupling_planner')
        
        self.declare_parameter("timer_period_seconds", 0.2)
        
        #services
        self.planner_mode_srv = self.create_service(CouplingPlannerMode,"planner_mode_request",self.planner_mode_service_callback)

        #subscriber
        self.ego_subscription = self.create_subscription(Ego,"/loc/lelo/truck",self.ego_vx_subscriber_callback,10)
        self.ego_curvature_subscription = self.create_subscription(AutoboxVehicle,"/AutoboxVehicle",self.ego_curvature_subscriber_callback,10)
        self.kingpin_pose_subscription = self.create_subscription(PoseStamped,"/pln/kingpin_emulation",self.kingpin_pose_subscriber_callback,10)

        #publisher
        self.trajectory_publisher = self.create_publisher(ControllerTrajectory, "/pln/gpu/trajectory",10)

        #timer
        self.timer_init = self.create_timer(2.5,self.init)

        #transformation
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer,self)

        #planner
        self.planner = CouplingPlanner( path_res=0.01, path23_res=0.2, vx=-0.5, acc_dec_time=2, history_point_limit=3, trajectory_backup=1,
                                        ego_delta_bilevel=0.5, goal_delta_bilevel=0.5, max_curvature=0.5, min_traj_length=2,max_traj_length=100,
                                        dis_prekingpin_kingpin=0
                                        )
            
        self.ego_pose = Pose()
        self.kingpin_pose = Pose()

        self.ego_pose_truckodom = PoseStamped()
        self.kingpin_pose_truckodom = PoseStamped()

        self.ego_msg = Ego()
        self.autoboxvehicle_msg = AutoboxVehicle()
        

    def init(self):
        self.get_logger().info("init")
        self.timer_init.cancel()
        self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle)

    def cycle(self):

        self.pose2planner()

        self.planner.visualization()
        self.planner.cycle()

        self.publish_controller_trajectory_msg()

    def pose2planner(self):
 
        self.give_ego_in_odom()

        #validate age of ego_pose, kingpin_pose, ego_vx
        age_ego = self.get_clock().now().nanoseconds/1e9 - self.ego_pose_truckodom.header.stamp.nanosec/1e9
        #age_kingpin = self.get_clock().now().nanoseconds/1e9 - self.kingpin_pose_truckodom.header.stamp.nanosec/1e9
        #age_ego_vx = self.get_clock().now().nanoseconds/1e9 - self.ego_msg.header.stamp.nanosec/1e9

        #if 0 < age_ego <= 5: #and 0 < age_kingpin <= 1 and 0 < age_ego_vx <= 1:

        self.planner.planner_mode = PlannerMode.COUPLING_PHASE_PREKINGPIN
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
        
        self.ego_pose.vx = self.ego_msg.velocity.linear.x
        self.ego_pose.curvature = 0.0#np.tan(self.autoboxvehicle_msg.wheel_angle)/3.6

        #convert PoseStamped KingPin into PlannerPose
        self.kingpin_pose.x = self.kingpin_pose_truckodom.pose.position.x
        self.kingpin_pose.y = self.kingpin_pose_truckodom.pose.position.y
        q = Quaternion( self.kingpin_pose_truckodom.pose.orientation.w,
                        self.kingpin_pose_truckodom.pose.orientation.x,
                        self.kingpin_pose_truckodom.pose.orientation.y,
                        self.kingpin_pose_truckodom.pose.orientation.z
                        )
        roll, pitch, yaw = q.to_euler()
        self.kingpin_pose.yaw = yaw
        
        self.kingpin_pose.vx = 0.0
        self.kingpin_pose.curvature = 0.0
        
        #update planner pose
        self.planner.update_pose(self.ego_pose,self.kingpin_pose)

        #else:
        #    self.planner.planner_mode = PlannerMode.STANDSTILL
        #    self.get_logger().info("ego_pose/ego_vx/kingpin_pose age not valid")

    def planner_mode_service_callback(self, request, response):

        if request.planner_mode is request.COUPLING_PHASE_STANDSTILL:
            self.planner.planner_mode = PlannerMode.STANDSTILL
        elif request.planner_mode is request.COUPLING_PHASE_TILL_PREKINGPIN:
            self.planner.planner_mode = PlannerMode.COUPLING_PHASE_PREKINGPIN
        elif request.planner_mode is request.COUPLING_PHASE_TILL_KINGPIN:
            self.planner.planner_mode = PlannerMode.COUPLING_PHASE_KINGPIN
        
        response.success = True 

    def ego_vx_subscriber_callback(self,ego_msg):
        self.ego_msg = ego_msg
        self.get_logger().info("got new ego_vx")
    
    def ego_curvature_subscriber_callback(self,autobox_msg):
        self.ego_pose.curvature = np.tan(autobox_msg.wheel_angle)/3.6
        self.get_logger().info("got new ego_curvature")

    def kingpin_pose_subscriber_callback(self, kingpin_pose_msg):
        self.transform_base2odom(kingpin_pose_msg)
        #self.transform_kingpin(pose_stamped_msg)
        self.get_logger().info("got new kingpin_pose")
        
    def give_ego_in_odom(self):

        now = self.get_clock().now()

        try:
            transform = self.transform_buffer.lookup_transform("truck_odom", "truck_base", Time())
            self.get_logger().info("ego_in_odom: x: {}, y: {}".format(transform.transform.translation.x, transform.transform.translation.y))
            
            self.ego_pose_truckodom.header.stamp = now.to_msg()
            self.ego_pose_truckodom.header.frame_id = "truck_odom"
            self.ego_pose_truckodom.pose.position.x = transform.transform.translation.x
            self.ego_pose_truckodom.pose.position.y = transform.transform.translation.y
            self.ego_pose_truckodom.pose.position.z = transform.transform.translation.z
            self.ego_pose_truckodom.pose.orientation.w = transform.transform.rotation.w
            self.ego_pose_truckodom.pose.orientation.x = transform.transform.rotation.x
            self.ego_pose_truckodom.pose.orientation.y = transform.transform.rotation.y
            self.ego_pose_truckodom.pose.orientation.z = transform.transform.rotation.z

        except LookupException:
            self.get_logger().info("no tf data - ego_in_odom")

    def transform_base2odom(self, kingpin_pose_msg):

        now = self.get_clock().now()
        try:
            transform = self.transform_buffer.lookup_transform("truck_odom", "truck_base", Time())
            #self.get_logger().info("Trafo: global2odom: x: {}, y: {}".format(self.tf_global2truckodom.transform.translation.x, self.tf_global2truckodom.transform.translation.y))
            self.kingpin_pose_truckodom = do_transform_pose_stamped(kingpin_pose_msg,transform)
            self.get_logger().info("kingpin@odom: x: {}, y: {}".format(self.kingpin_pose_truckodom.pose.position.x, self.kingpin_pose_truckodom.pose.position.y))
        except LookupException:
            self.get_logger().info("no tf data - global2odom")

    def publish_controller_trajectory_msg(self):

        now = self.get_clock().now()
        msg = ControllerTrajectory()
        msg.header.stamp = now.to_msg()
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
            self.trajectory_publisher.publish(msg)
        else:
            self.get_logger().info("wrong length of trajectory")

def main(args=None):
    rclpy.init(args=args)
    coupling_planner_node = CouplingPlannerNode()
    rclpy.spin(coupling_planner_node)
    coupling_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
