import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, LookupException, Time, Duration
from squaternion import Quaternion

from geometry_msgs.msg import  TransformStamped, PoseStamped

from .kingpin_emulation_tools import *

class KingpinEmulationNode(Node):
 
    def __init__(self):
        super().__init__('pln_kingpin_emulation')
        
        self.declare_parameter("timer_period_seconds", 0.2)

        self.kingpin_pose_publisher = self.create_publisher(PoseStamped, "/pln/kingpin_emulation",10)

        self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle)

        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer,self)

        self.tf_global2truckodom = TransformStamped()
        self.tf_global2truckbase = TransformStamped()

        self.kingpin_pose_global = PoseStamped()
        self.kingpin_pose_truckbase = PoseStamped()
        self.kingpin_pose_truckodom = PoseStamped()

        self.transform_ego_in_global()


    def cycle(self):
        self.get_logger().info("cycle")

        self.get_logger().info("ego@global: x: {}, y: {}".format(self.kingpin_pose_global.pose.position.x, self.kingpin_pose_global.pose.position.y))

        self.lookup_tf_global2odom()
        #self.lookup_tf_global2base()
    
        self.publish_kingpin_pose_msg()

    def transform_ego_in_global(self):

        now = self.get_clock().now()

        try:
            transform = self.transform_buffer.lookup_transform("global", "truck_base", Time())
            self.get_logger().info("ego@gobal: x: {}, y: {}".format(transform.transform.translation.x, transform.transform.translation.y))
            
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

    def transform_ego_in_odom(self):

        now = self.get_clock().now()

        try:
            transform = self.transform_buffer.lookup_transform("truck_odom", "truck_base", Time())
            self.get_logger().info("ego@odom: x: {}, y: {}".format(transform.transform.translation.x, transform.transform.translation.y))
            
            self.kingpin_pose_truckodom.header.stamp = now.to_msg()
            self.kingpin_pose_truckodom.header.frame_id = "truck_odom"

            self.kingpin_pose_truckodom.pose.position.x = transform.transform.translation.x
            self.kingpin_pose_truckodom.pose.position.y = transform.transform.translation.y
            self.kingpin_pose_truckodom.pose.position.z = transform.transform.translation.z

            self.kingpin_pose_truckodom.pose.orientation.w = transform.transform.rotation.w
            self.kingpin_pose_truckodom.pose.orientation.x = transform.transform.rotation.x
            self.kingpin_pose_truckodom.pose.orientation.y = transform.transform.rotation.y
            self.kingpin_pose_truckodom.pose.orientation.z = transform.transform.rotation.z

        except LookupException:
            self.get_logger().info("no tf data - ego_in_odom")


    def lookup_tf_global2odom(self):

        now = self.get_clock().now()
        try:
            self.tf_global2truckodom = self.transform_buffer.lookup_transform("truck_odom", "global", Time())
            #self.get_logger().info("Trafo: global2odom: x: {}, y: {}".format(self.tf_global2truckodom.transform.translation.x, self.tf_global2truckodom.transform.translation.y))
            self.kingpin_pose_truckodom = do_transform_pose_stamped(self.kingpin_pose_global,self.tf_global2truckodom)
            self.get_logger().info("ego@odom: x: {}, y: {}".format(self.kingpin_pose_truckodom.pose.position.x, self.kingpin_pose_truckodom.pose.position.y))
        except LookupException:
            self.get_logger().info("no tf data - global2odom")

    def lookup_tf_global2base(self):

        now = self.get_clock().now()
        try:
            self.tf_global2truckbase = self.transform_buffer.lookup_transform("truck_base", "global", Time())
            #self.get_logger().info("Trafo: global2base: x: {}, y: {}".format(self.tf_global2truckbase.transform.translation.x, self.tf_global2truckbase.transform.translation.y))
            self.kingpin_pose_truckbase = do_transform_pose_stamped(self.kingpin_pose_global,self.tf_global2truckbase)
            self.get_logger().info("ego@base: x: {}, y: {}".format(self.kingpin_pose_truckbase.pose.position.x, self.kingpin_pose_truckbase.pose.position.y))
        except LookupException:
            self.get_logger().info("no tf data - global2base")


    def transform_kingpin_2truckbase(self):
        try:
            self.kingpin_pose_truckbase = self.transform_buffer.transform(self.kingpin_pose_global,"truck_base")
            self.get_logger().info("kingpin@truck: x: {}, y: {}".format(self.kingpin_pose_truckbase.x, self.kingpin_pose_truckbase.y))
        except:
            self.get_logger().info("no tf data - kingpin2truckbase")
            
    def transform_kingpin_2truckodom(self):
        try:
            self.kingpin_pose_truckodom = self.transform_buffer.transform(self.kingpin_pose_global,"truck_odom")
            self.get_logger().info("kingpin@odom: x: {}, y: {}".format(self.kingpin_pose_truckodom.x, self.kingpin_pose_truckodom.y))
        except:
            self.get_logger().info("no tf data - kingpin2truckodom")

    def publish_kingpin_pose_msg(self):
        self.kingpin_pose_publisher.publish(self.kingpin_pose_truckodom)


def main(args=None):
    rclpy.init(args=args)
    kingpin_emulation_node = KingpinEmulationNode()
    rclpy.spin(kingpin_emulation_node)
    kingpin_emulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
