import time
import numpy as np

from coupling_planner_lib import  Pose, TrajectoryPoint, PlannerMode, CouplingPlanner

class Simulation:

    def __init__(self):
        
        self.planner = CouplingPlanner( path_res=0.1, path23_res=0.05, vx=-0.41, acc_dec_time=1, history_point_limit=3, trajectory_backup=1,
                                        ego_delta_bilevel=0.3, goal_delta_bilevel=0.15, max_curvature=3, min_traj_length=2,max_traj_length=500,
                                        dis_prekingpin_kingpin=2
                                        )
            
        self.planner.planner_mode = PlannerMode.SIMULATION

    def simulate(self):
    
        counter = 0

        while True:
            print("cycle: ",counter,end=' -> ')
            counter += 1
            self.planner.cycle()
            time.sleep(0.2)
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()