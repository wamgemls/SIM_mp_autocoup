import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener,TransformBroadcaster, Buffer, LookupException, Time, Duration
from squaternion import Quaternion

from goetting_trailer_interfaces.msg import Activation,TrailerDetection

from geometry_msgs.msg import  TransformStamped, PoseStamped

from .tf_tools import *
from .tools import *

class KingpinBroadcasterNode(Node):
 
    def __init__(self):
        super().__init__('pln_kingpin_broadcaster')

        self.declare_parameter("timer_period_seconds", 0.1)
        self.declare_parameter("noise_kingpin_pose", False)

        #subscriber
        self.kingpin_pose_subscription = self.create_subscription(PoseStamped,"/pln/kingpin_gui",self.kingpin_pose_subscriber_callback,10)

        #publisher
        self.kingpin_pose_pub = self.create_publisher(PoseStamped,"/pln/kingpin_planner",10)
        
        #timer
        self.timer_init = self.create_timer(2.5,self.init)

        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self.transform_buffer,self)

        self.tf_global2truckodom = TransformStamped()
        self.tf_truckodom2truckbase = TransformStamped()
  
        self.kingpin_pose_global_emulation = PoseStamped()
        self.kingpin_pose_truckbase_emulation = PoseStamped()

    def init(self):
        self.timer_init.cancel()
        self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.emulation_cycle)

    def emulation_cycle(self):
        self.get_logger().info("Emulation - Cycle")
        self.get_logger().info("kingpin@global: x: {}, y: {}".format(   self.kingpin_pose_global_emulation.pose.position.x, 
                                                                        self.kingpin_pose_global_emulation.pose.position.y))

        self.tf_read()
        self.transform_kingpin_global2base()

        self.get_logger().info("kingpin@truckbase: x: {}, y: {}".format(self.kingpin_pose_truckbase_emulation.pose.position.x, 
                                                                        self.kingpin_pose_truckbase_emulation.pose.position.y
                                                                        ))

        if self.get_parameter("noise_kingpin_pose").get_parameter_value().bool_value:
            self.kingpin_pose_truckbase_emulation = noise_pose_stamped(self.kingpin_pose_truckbase_emulation)

        self.kingpin_pose_pub.publish(self.kingpin_pose_truckbase_emulation)
    
    def tf_read(self):
        
        try:
            self.tf_global2truckodom = self.transform_buffer.lookup_transform("truck_odom", "global", Time())
        except LookupException:
            self.get_logger().info("ERROR - no tf data - global2truckodom")
        try:
            self.tf_truckodom2truckbase = self.transform_buffer.lookup_transform("truck_base", "truck_odom", Time())
        except LookupException:
            self.get_logger().info("ERROR - no tf data - truckodom2truckbase")

    def transform_kingpin_global2base(self):
        self.kingpin_pose_truckbase_emulation = do_transform_pose_stamped(self.kingpin_pose_global_emulation,
            combine_transform(self.tf_global2truckodom,self.tf_truckodom2truckbase))

    def kingpin_pose_subscriber_callback(self, kingpin_pose_msg):
        self.kingpin_pose_global_emulation = do_transform_pose_stamped(kingpin_pose_msg,
            combine_transform(invert_transform(self.tf_truckodom2truckbase),invert_transform(self.tf_global2truckodom)))
            
def main(args=None):
    rclpy.init(args=args)
    kingpin_broadcaster_node = KingpinBroadcasterNode()
    rclpy.spin(kingpin_broadcaster_node)
    kingpin_broadcaster_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
