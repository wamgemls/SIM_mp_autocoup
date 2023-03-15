import time

from tools import  Pose, TrajectoryPoint, PlannerMode, CouplingPlanner

from viz_tools import *

class Simulation:

    def __init__(self):
        
        self.planner = CouplingPlanner( path_res=0.01, 
                                        path23_res=0.1,
                                        vx=2.0, 
                                        acc_time=3.0,
                                        dec_time=3.0, 
                                        history_point_limit=3, 
                                        trajectory_backup=0,
                                        ego_delta_bilevel=0.5, 
                                        goal_delta_bilevel=0.5, 
                                        max_curvature=0.4, 
                                        min_traj_length=2,
                                        max_traj_length=50,
                                        dis_prekingpin_kingpin=0.529
                                        )
        
        app = QtWidgets.QApplication(sys.argv)
        self.gui = PlannerGUI()
        self.gui.show()

    def simulate(self):
    
        counter = 0

        while True:

            print("cycle: ",counter)
            counter += 1
            self.transfer_input_data()
            self.full_draw()
            self.planner.prekingpin()
            self.planner.ego_drive_step(self.planner.trajectory23)
            #time.sleep(0.2)

    def transfer_input_data(self):

        self.gui.planner_data.data_transfer(self.planner.trajectory, self.planner.trajectory23,
                                            self.planner.ego_pose.x, self.planner.ego_pose.y, self.planner.ego_pose.yaw,self.planner.ego_pose.vx,self.planner.ego_pose.curvature,
                                            self.planner.kingpin_goal_pose.x,self.planner.kingpin_goal_pose.y, self.planner.kingpin_goal_pose.yaw,
                                            self.planner.prekingpin_goal_pose.x, self.planner.prekingpin_goal_pose.y, self.planner.prekingpin_goal_pose.yaw
                                            )
    
    def full_draw(self):
        self.gui.full_update_data_bird_fig()
        self.gui.draw_bird_figure()
        self.gui.update_data_graph_fig()
        self.gui.draw_graph_figure()
        self.gui.update_data_graph23_fig()
        self.gui.draw_graph23_figure()
        self.gui.update_status_screen()
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()