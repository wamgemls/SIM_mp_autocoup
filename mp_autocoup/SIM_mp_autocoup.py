import time
import numpy as np

from LIB_mp_autocoup import  Pose, TrajectoryPoint, PlannerMode, AutocoupPlanner

class Simulation:

    def __init__(self):
        
        self.planner = AutocoupPlanner( path_res=0.01, path23_res=0.1, vx=-0.41, acc_dec_time=0.5, history_point_limit=3, trajectory_backup=1,
                                        ego_delta_bilevel=0.5, goal_delta_bilevel=0.15, max_curvature=0.26, min_traj_length=2,
                                        dis_prekingpin_kingpin=2
                                        )
            
        self.ego_pose = Pose(None, 2, 10, np.deg2rad(190),0, 0)
        self.kingpin_pose = Pose(None, 15, 5, np.deg2rad(140), 0, 0)
        
        self.planner.update_pose(self.ego_pose,self.kingpin_pose)

    def simulate(self):
    
        counter = 0

        while True:

            print("cycle: ",counter,end=' -> ')
            counter += 1

            #self.planner.update_pose(self.init_pose,self.ego_pose,self.kingpin_pose)

            self.planner.cycle()

            time.sleep(0.5)
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()