import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, TransformStamped, Buffer, LookupException, Time, Duration
from squaternion import Quaternion

from geometry_msgs.msg import PoseStamped

from pln_interfaces.msg import ControllerTrajectory, ControllerTrajectoryPoint
from pln_interfaces.msg import Ego

class KingpinEmulationNode(Node):
 
    def __init__(self):
        super().__init__('pln_kingpin_emulation')
        
        self.declare_parameter("timer_period_seconds", 0.2)

        self.kingpin_pose__truckodom_publisher = self.create_publisher(PoseStamped, "/pln/kingpin_emulation_truckodom",10)
        self.kingpin_pose__truckbase_publisher = self.create_publisher(PoseStamped, "/pln/kingpin_emulation_truckbase",10)

        self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle)

        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer,self)

        self.kingpin_pose_global = PoseStamped()
        self.kingpin_pose_base = PoseStamped()
        self.kingpin_pose_odom = PoseStamped()

        self.kingpin_pose_notset = True
        
    def cycle(self):
        self.get_logger().info("new cycle")

        while self.kingpin_pose_notset:
            print("Waiting for KingPin Pose - Press Enter")
            input()
            self.transform_ego2global()
            self.kingpin_pose_notset = False

        self.transform_kingpin_global2truckbase()
        self.transform_kingpin_global2truckodom()

        self.publish_kingpin_pose_msg()

    def transform_ego2global(self):

        now = self.get_clock().now()

        try:
            transform = self.transform_buffer.lookup_transform("global", "truck_base", Time())
            self.get_logger().info("We have this ego in global data: t: {}.{}, x: {}, y: {}".format(
                                    transform.header.stamp.sec, transform.header.stamp.nanosec,
                                    transform.transform.translation.x, transform.transform.translation.y))
            
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
            self.get_logger().info("no tf data - global2base")

    def transform_kingpin_global2truckbase(self):
        try:
            self.kingpin_pose_base = self.transform_buffer.transform(self.kingpin_pose_global,"truck_base")
        except:
            self.get_logger().info("no tf data - global2base")
            

    def transform_kingpin_global2truckodom(self):
        try:
            self.kingpin_pose_odom = self.transform_buffer.transform(self.kingpin_pose_global,"truck_odom")
        except:
            self.get_logger().info("no tf data - global2odom")


    def publish_kingpin_pose_msg(self):
        self.kingpin_pose__truckbase_publisher.publish(self.kingpin_pose_base)
        self.kingpin_pose__truckodom_publisher.publish(self.kingpin_pose_odom)


def main(args=None):
    rclpy.init(args=args)
    kingpin_emulation_node = KingpinEmulationNode()
    rclpy.spin(kingpin_emulation_node)
    kingpin_emulation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
