import numpy as np
import time

import rclpy
from rclpy.node import Node
from squaternion import Quaternion

from geometry_msgs.msg import PoseStamped
from pln_interfaces.msg import ControllerTrajectoryPoint, CouplingPlannerData

from .tools import AutocoupAnimation
from coupling_planner.tools import TrajectoryPoint,Pose

class CouplingPlannerVisualization(Node):
 
    def __init__(self):
        super().__init__('pln_visualization')
        
        self.declare_parameter("timer_period_seconds", 0.5)
        
        #subscriber
        self.CouplingPlannerData_subscription = self.create_subscription(CouplingPlannerData,"/pln/coupling_planner_data",self.CouplingPlannerData_callback,10)

        #timer
        self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle)


        self.animation = AutocoupAnimation()
        self.coupling_planner_data = CouplingPlannerData()

        self.trajectory_p1 = []
        self.trajectory_p2 = []
        self.trajectory23 = []

        self.ego_pose = Pose()
        self.prekingpin_pose = Pose()
        self.kingpin_pose = Pose()

    def cycle(self):
        starttime = self.get_clock().now().nanoseconds /1e9
        
        self.read_input_data()
        self.visualization()

        endtime = self.get_clock().now().nanoseconds /1e9

        self.get_logger().info("Cycletime: {}".format(endtime-starttime))

    def CouplingPlannerData_callback(self,coupling_planner_data_input):
        self.coupling_planner_data = coupling_planner_data_input

    def read_input_data(self):
        self.ego_pose = self.conversion_posestamped2pose(self.coupling_planner_data.ego)
        self.prekingpin_pose = self.conversion_posestamped2pose(self.coupling_planner_data.prekingpin)
        self.kingpin_pose = self.conversion_posestamped2pose(self.coupling_planner_data.kingpin)

        self.trajectory_p1 = self.conversion_controllertrajectory2trajectory(self.coupling_planner_data.trajectory_prekingpin)
        self.trajectory_p2 = self.conversion_controllertrajectory2trajectory(self.coupling_planner_data.trajectory_kingpin)
        self.trajectory23 = self.conversion_controllertrajectory2trajectory(self.coupling_planner_data.trajectory23)

    def conversion_controllertrajectory2trajectory(self,controllertrajectorylist):
        trajectory = []
        for controllertrajectorypoint in controllertrajectorylist:
            trajectory.append( TrajectoryPoint( t=controllertrajectorypoint.t,
                                                s=controllertrajectorypoint.s,
                                                x=controllertrajectorypoint.x,
                                                y=controllertrajectorypoint.y,
                                                vx=controllertrajectorypoint.vx,
                                                ax=controllertrajectorypoint.ax,
                                                yaw=controllertrajectorypoint.yaw,
                                                curvature=controllertrajectorypoint.curvature                                                
                                                ))

        return trajectory

    def conversion_posestamped2pose(self,posestamped):
        pose = Pose()
        pose.x = posestamped.pose.position.x
        pose.y = posestamped.pose.position.y
        q = Quaternion( posestamped.pose.orientation.w,
                        posestamped.pose.orientation.x,
                        posestamped.pose.orientation.y,
                        posestamped.pose.orientation.z
                        )
        roll, pitch, yaw = q.to_euler()
        pose.yaw = yaw

        return pose

    def visualization(self):
        self.animation.data_transfer(   self.trajectory_p1, self.trajectory_p2, self.trajectory23,
                                        self.ego_pose.x,self.ego_pose.y,self.ego_pose.yaw,
                                        self.kingpin_pose.x,self.kingpin_pose.y,self.kingpin_pose.yaw,
                                        self.prekingpin_pose.x,self.prekingpin_pose.y,self.prekingpin_pose.yaw)

def main(args=None):
    rclpy.init(args=args)
    coupling_planner_visualization = CouplingPlannerVisualization()
    rclpy.spin(coupling_planner_visualization)
    coupling_planner_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
