import numpy as np
import time
import sys

import rclpy
from rclpy.node import Node
from squaternion import Quaternion

from geometry_msgs.msg import PoseStamped
from pln_interfaces.msg import ControllerTrajectoryPoint, CouplingPlannerData

from .scope import *
from coupling_planner.planner import TrajectoryPoint,Pose

class CouplingScopeNode(Node):
 
    def __init__(self):
        super().__init__('pln_coupling_scope')
        
        self.declare_parameter("timer_period_seconds", 0.2)
        self.declare_parameter("csv_save", False)
        self.declare_parameter("csv_path", "/workspace/src/trailer_coupling/coupling_planner_logging/csv")
        self.declare_parameter("csv_filename", "dummy")
        
        #publisher
        self.kingpin_pose_pub = self.create_publisher(PoseStamped,"/pln/kingpin_gui",10)

        #subscriber
        self.CouplingPlannerData_subscription = self.create_subscription(CouplingPlannerData,"/pln/coupling_planner_data",self.coupling_planner_data_callback,10)

        #timer
        self.timer = self.create_timer(self.get_parameter("timer_period_seconds").get_parameter_value().double_value,self.cycle)
        
        self.coupling_planner_data = CouplingPlannerData()
        
        app = QtWidgets.QApplication(sys.argv)
        self.gui = CouplingScope()
        self.gui.show()


    def cycle(self):
        starttime = self.get_clock().now().nanoseconds/1e9
        
        self.transfer_input_data()
        self.full_draw()
        self.kingpin_pose_publisher()

        endtime = self.get_clock().now().nanoseconds/1e9
        self.get_logger().info("Full Cycletime: {}".format(endtime-starttime))

    def coupling_planner_data_callback(self,coupling_planner_data_input):
        self.coupling_planner_data = coupling_planner_data_input

    def transfer_input_data(self):
        timestamp = self.coupling_planner_data.header.stamp.sec + (self.coupling_planner_data.header.stamp.nanosec / 1e9)

        ego_pose = self.conversion_posestamped2pose(self.coupling_planner_data.ego_pose)
        kingpin_goal_pose = self.conversion_posestamped2pose(self.coupling_planner_data.kingpin_goal_pose)
        prekingpin_goal_pose = self.conversion_posestamped2pose(self.coupling_planner_data.prekingpin_goal_pose)

        trajectory = self.conversion_controllertrajectory2trajectory(self.coupling_planner_data.trajectory)
        trajectory23 = self.conversion_controllertrajectory2trajectory(self.coupling_planner_data.trajectory23)

        ego_pose.vx = self.coupling_planner_data.ego_vx
        ego_pose.curvature = self.coupling_planner_data.ego_curvature

        self.gui.planner_data.data_transfer(trajectory, trajectory23,
                                            ego_pose.x, ego_pose.y, ego_pose.yaw,ego_pose.vx,ego_pose.curvature,
                                            kingpin_goal_pose.x,kingpin_goal_pose.y, kingpin_goal_pose.yaw,
                                            prekingpin_goal_pose.x, prekingpin_goal_pose.y, prekingpin_goal_pose.yaw
                                            )
        
        self.get_logger().info("Timestamp: {}".format(timestamp))

        if self.get_parameter("csv_save").get_parameter_value().bool_value:
            self.gui.planner_data.data2csv( timestamp,
                                                self.get_parameter("csv_path").get_parameter_value().string_value,
                                                    self.get_parameter("csv_filename").get_parameter_value().string_value)

    def full_draw(self):
        self.gui.full_update_data_bird_fig()
        self.gui.draw_bird_figure()
        self.gui.update_data_graph_fig()
        self.gui.draw_graph_figure()
        self.gui.update_data_graph23_fig()
        self.gui.draw_graph23_figure()
        self.gui.update_status_screen()

    def kingpin_pose_publisher(self):

        if self.gui.tab1.send_goal:

            kingpin_truckbase_posestamped = PoseStamped()
            kingpin_truckbase_posestamped.header.stamp = self.get_clock().now().to_msg()
            kingpin_truckbase_posestamped.header.frame_id = "truck_base"

            x = 0.0
            y = 0.0
            yaw = 0.0
            
            try:
                if self.gui.tab1.goal_x_input.text() != '':
                    x= float(self.gui.tab1.goal_x_input.text().strip("\'"))
            
                if self.gui.tab1.goal_y_input.text() != '':
                    y= float(self.gui.tab1.goal_y_input.text().strip("\'"))
                
                if self.gui.tab1.goal_yaw_input.text() != '':
                    yaw= np.deg2rad(float(self.gui.tab1.goal_yaw_input.text().strip("\'")))

            except:
                self.get_logger().info("invalid goal pose")

            kingpin_truckbase_posestamped.pose.position.x = x
            kingpin_truckbase_posestamped.pose.position.y = y
            kingpin_truckbase_posestamped.pose.position.z = 0.0
            
            
            q = Quaternion.from_euler(0.0,0.0,yaw)
            kingpin_truckbase_posestamped.pose.orientation.w = float(q.w)
            kingpin_truckbase_posestamped.pose.orientation.x = float(q.x)
            kingpin_truckbase_posestamped.pose.orientation.y = float(q.y)
            kingpin_truckbase_posestamped.pose.orientation.z = float(q.z)

            self.kingpin_pose_pub.publish(kingpin_truckbase_posestamped)

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


def main(args=None):
    
    rclpy.init(args=args)
    coupling_planner_visualization = CouplingScopeNode()
    rclpy.spin(coupling_planner_visualization)
    coupling_planner_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
