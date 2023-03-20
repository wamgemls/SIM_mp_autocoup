import time

from tools import  Pose, TrajectoryPoint, PlannerMode, CouplingPlanner

from tools_viz import AutocoupAnimation

class Simulation:

    def __init__(self):
        
        self.planner = CouplingPlanner( path_res=0.1, 
                                        path23_res=0.17, 
                                        vx=2.0, 
                                        acc_time=3.0,
                                        dec_time=3.0, 
                                        history_point_limit=3, 
                                        trajectory_backup=0,
                                        ego_delta_bilevel=0.3, 
                                        goal_delta_bilevel=0.15, 
                                        max_curvature=0.3, 
                                        min_traj_length=2,
                                        max_traj_length=50,
                                        dis_prekingpin_kingpin=0.0
                                        )

        self.animation = AutocoupAnimation()

    def simulate(self):
    
        counter = 0

        while True:

            print("cycle: ",counter)
            counter += 1
            self.data_transfer()
            self.full_draw()
            self.planner.prekingpin()
            #time.sleep(0.2)

    def data_transfer(self):
        self.animation.data_transfer(   self.planner.trajectory,self.planner.trajectory23,\
                                        self.planner.ego_pose.x,self.planner.ego_pose.y,self.planner.ego_pose.yaw,\
                                        self.planner.kingpin_goal_pose.x,self.planner.kingpin_goal_pose.y,self.planner.kingpin_goal_pose.yaw,\
                                        self.planner.prekingpin_goal_pose.x,self.planner.prekingpin_goal_pose.y,self.planner.prekingpin_goal_pose.yaw)

    def full_draw(self):
        self.animation.full_draw()
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()