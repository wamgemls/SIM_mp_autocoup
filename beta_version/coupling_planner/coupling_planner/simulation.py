import time

from tools import  Pose, TrajectoryPoint, PlannerMode, CouplingPlanner

from planner_visualization.tools import AutocoupAnimation

class Simulation:

    def __init__(self):
        
        self.planner = CouplingPlanner( path_res=0.1, 
                                        path23_res=0.17, 
                                        vx=-0.55, 
                                        acc_time=0.5,
                                        dec_time=0.5, 
                                        history_point_limit=3, 
                                        trajectory_backup=1,
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
        self.planner.planner_mode = PlannerMode.SIMULATION

        while True:

            print("cycle: ",counter)
            counter += 1
            self.data_transfer()
            self.full_draw()
            self.planner.cycle()
            #time.sleep(0.2)

    def data_transfer(self):
        self.animation.data_transfer(   self.planner.trajectory_p1,self.planner.trajectory_p2,self.planner.trajectory23,\
                                        self.planner.ego_pose.x,self.planner.ego_pose.y,self.planner.ego_pose.yaw,\
                                        self.planner.kingpin_pose.x,self.planner.kingpin_pose.y,self.planner.kingpin_pose.yaw,\
                                        self.planner.prekingpin_pose.x,self.planner.prekingpin_pose.y,self.planner.prekingpin_pose.yaw)

    def full_draw(self):
        self.animation.full_draw()
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()