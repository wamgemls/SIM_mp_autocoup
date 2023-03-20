import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener,TransformBroadcaster, Buffer, LookupException, Time, Duration
from squaternion import Quaternion

from goetting_trailer_interfaces.msg import Activation,TrailerDetection

from geometry_msgs.msg import  TransformStamped, PoseStamped

from .tools import *

class KingpinBroadcasterNode(Node):
 
    def __init__(self):
        super().__init__('pln_kingpin_broadcaster')
        
        self.declare_parameter("emulation", True)
        self.declare_parameter("noise_kingpin_pose", False)
        self.declare_parameter("timer_period_seconds", 0.1)

        #subscriber
        self.trailerdetection_subscription = self.create_subscription(TrailerDetection,"/unknown",self.trailerdetection_callback,10)

        #publisher
        self.kingpin_pose_pub = self.create_publisher(PoseStamped, "/pln/kingpin_emulation",10)
        
        #timer
        self.timer_init = self.create_timer(2.5,self.init)


        self.transform_broadcaster = TransformBroadcaster(self)
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer,self)

        self.kingpin_pose_global = PoseStamped()
        self.kingpin_pose_truckbase_emulation = PoseStamped()
        self.kingpin_pose_truckodom_emulation = PoseStamped()


        self.kingpin_pose_truckbase_goetting = PoseStamped()

    def init(self):
        self.timer_init.cancel()

        if self.get_parameter("emulation").get_parameter_value().bool_value:
            self.store_ego_in_global()
            self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.emulation_cycle)
        else:
            self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.goetting_cycle)

    def emulation_cycle(self):
        self.get_logger().info("Emulation - Cycle")
        self.get_logger().info("kingpin@global: x: {}, y: {}".format(   self.kingpin_pose_global.pose.position.x, 
                                                                        self.kingpin_pose_global.pose.position.y))
        
        self.transform_kingpin_global2base()
        self.transform_kingpin_global2odom()

        if self.get_parameter("noise_kingpin_pose").get_parameter_value().bool_value:
            self.kingpin_pose_truckbase_emulation = noise_pose_stamped(self.kingpin_pose_truckbase_emulation)

        self.broadcast_kingpin_frame()
        self.publish_kingpin_pose(self.kingpin_pose_truckodom_emulation)
    
    def goetting_cycle(self):
        self.get_logger().info("Goetting - Cycle")
        self.get_logger().info("kingpin@truckbase: x: {}, y: {}".format(self.kingpin_pose_truckbase_goetting.pose.position.x, 
                                                                        self.kingpin_pose_truckbase_goetting.pose.position.y))

        self.broadcast_kingpin_frame(self.kingpin_pose_truckbase_goetting)

    def store_ego_in_global(self):

        now = self.get_clock().now()
        try:
            transform = self.transform_buffer.lookup_transform("global", "truck_base", Time())
            self.get_logger().info("kingpin@gobal: x: {}, y: {}".format(transform.transform.translation.x, 
                                                                        transform.transform.translation.y))
            
            self.kingpin_pose_global.header.stamp = now.to_msg()
            self.kingpin_pose_global.header.frame_id = "global"

            self.kingpin_pose_global.pose.position.x = transform.transform.translation.x
            self.kingpin_pose_global.pose.position.y = transform.transform.translation.y
            self.kingpin_pose_global.pose.position.z = transform.transform.translation.z

            self.kingpin_pose_global.pose.orientation.w = transform.transform.rotation.w
            self.kingpin_pose_global.pose.orientation.x = transform.transform.rotation.x
            self.kingpin_pose_global.pose.orientation.y = transform.transform.rotation.y
            self.kingpin_pose_global.pose.orientation.z = transform.transform.rotation.z

        except LookupException:
            self.get_logger().info("no tf data - ego_in_gobal")

    def transform_kingpin_global2base(self):

        try:
            transform = self.transform_buffer.lookup_transform("truck_base", "global", Time())
            self.kingpin_pose_truckbase_emulation = do_transform_pose_stamped(self.kingpin_pose_global,transform)
            
            self.get_logger().info("kingpin@truckbase: x: {}, y: {}".format(    self.kingpin_pose_truckbase_emulation.pose.position.x, 
                                                                                self.kingpin_pose_truckbase_emulation.pose.position.y
                                                                                ))
        except LookupException:
            self.get_logger().info("no tf data - global2base")

    def transform_kingpin_global2odom(self):

        try:
            transform = self.transform_buffer.lookup_transform("truck_odom", "global", Time())
            self.kingpin_pose_truckodom_emulation = do_transform_pose_stamped(self.kingpin_pose_global,transform)
            
            self.get_logger().info("kingpin@truckodom: x: {}, y: {}".format(    self.kingpin_pose_truckodom_emulation.pose.position.x, 
                                                                                self.kingpin_pose_truckodom_emulation.pose.position.y
                                                                                ))
        except LookupException:
            self.get_logger().info("no tf data - global2odom")

    def broadcast_kingpin_frame(self):
        # Create a new transform from the parent frame to the new frame
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "truck_base"
        transform.child_frame_id = "kingpin_base"
        transform.transform.translation.x = self.kingpin_pose_truckbase_emulation.pose.position.x
        transform.transform.translation.y = self.kingpin_pose_truckbase_emulation.pose.position.y
        transform.transform.translation.z = self.kingpin_pose_truckbase_emulation.pose.position.z

        transform.transform.rotation.w = self.kingpin_pose_truckbase_emulation.pose.orientation.w
        transform.transform.rotation.x = self.kingpin_pose_truckbase_emulation.pose.orientation.x
        transform.transform.rotation.y = self.kingpin_pose_truckbase_emulation.pose.orientation.y
        transform.transform.rotation.z = self.kingpin_pose_truckbase_emulation.pose.orientation.z

        # Publish the new transform as a dynamic transform
        self.transform_broadcaster.sendTransform(transform)

    def trailerdetection_callback(self,trailerdetection_input):
        self.kingpin_pose_truckbase_goetting = trailerdetection_input.pose

    def publish_kingpin_pose(self,pose_stamped):
        self.kingpin_pose_pub.publish(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    kingpin_broadcaster_node = KingpinBroadcasterNode()
    rclpy.spin(kingpin_broadcaster_node)
    kingpin_broadcaster_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
