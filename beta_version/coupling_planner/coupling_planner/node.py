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
from kingpin_emulation.tools import do_transform_pose_stamped

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
        self.controller_trajectory_publisher = self.create_publisher(ControllerTrajectory, "/pln/gpu/trajectory",10)
        self.coupling_planner_data_publisher = self.create_publisher(CouplingPlannerData,"/pln/coupling_planner_data",10)

        #timer
        self.timer_init = self.create_timer(2.5,self.init)

        #transformation
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer,self)

        #planner
        self.planner = CouplingPlanner( path_res=0.1, path23_res=0.1, vx=-0.41, acc_dec_time=2, history_point_limit=3, trajectory_backup=1,
                                        ego_delta_bilevel=0.5, goal_delta_bilevel=0.5, max_curvature=3, min_traj_length=2,max_traj_length=100,
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
        starttime = self.get_clock().now().nanoseconds /1e9
        
        self.pose2planner()
        self.planner.cycle()
        self.publish_controller_trajectory()
        self.publish_coupling_planner_data()

        endtime = self.get_clock().now().nanoseconds /1e9

        self.get_logger().info("Cycletime: {}".format(endtime-starttime))

    def pose2planner(self):
 
        self.give_ego_in_odom()

        #validate age of ego_pose, kingpin_pose, ego_vx
        now = self.get_clock().now().nanoseconds/1e9
        age_ego = now - self.ego_pose_truckodom.header.stamp.nanosec/1e9
        age_kingpin = now - self.kingpin_pose_truckodom.header.stamp.nanosec/1e9
        age_ego_vx = now - self.ego_msg.header.stamp.nanosec/1e9

        if 0 < age_ego <= 1 and 0 < age_kingpin <= 1 and 0 < age_ego_vx <= 1:

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

        else:
            self.planner.planner_mode = PlannerMode.SIMULATION
            self.get_logger().info("ego_pose/ego_vx/kingpin_pose age not valid")

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
            self.kingpin_pose_truckodom = do_transform_pose_stamped(kingpin_pose_msg,transform)
            self.get_logger().info("kingpin@odom: x: {}, y: {}".format(self.kingpin_pose_truckodom.pose.position.x, self.kingpin_pose_truckodom.pose.position.y))
        except LookupException:
            self.get_logger().info("no tf data - global2odom")

    def publish_controller_trajectory(self):

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
            self.controller_trajectory_publisher.publish(msg)
        else:
            self.get_logger().info("wrong length of trajectory")

    def publish_coupling_planner_data(self):

        now = self.get_clock().now()
        msg = CouplingPlannerData()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "truck_odom"

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

        now = self.get_clock().now()
        posestamped = PoseStamped()
        posestamped.header.stamp = now.to_msg()
        posestamped.header.frame_id = "truck_odom"
        
        posestamped.pose.position.x = float(pose.x)
        posestamped.pose.position.y = float(pose.y)
        posestamped.pose.position.z = 0.0
        
        q= Quaternion.from_euler(0.0,0.0,float(pose.yaw))
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
