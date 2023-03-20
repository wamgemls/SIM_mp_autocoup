import time
import numpy as np

from LIB_mp_autocoup import  Pose, TrajectoryPoint, PlannerMode, AutocoupPlanner

class Simulation:

    def __init__(self):
        
        self.planner = AutocoupPlanner( path_res=0.01, path23_res=0.1, vx=-0.41, acc_dec_time=0.5, history_point_limit=3, trajectory_backup=3,
                                        ego_delta_bilevel=0.5, goal_delta_bilevel=0.15, max_curvature=0.26, min_traj_length=2,max_traj_length=15,
                                        dis_prekingpin_kingpin=2
                                        )
            
        self.planner.planner_mode = PlannerMode.SIMULATION

    def simulate(self):
    
        counter = 0

        while True:

            print("cycle: ",counter,end=' -> ')
            counter += 1

            self.planner.cycle()

            time.sleep(0.5)
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()