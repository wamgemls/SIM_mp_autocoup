import time
import numpy as np

from LIB_mp_autocoup import AutocoupPlanner, Pose, TrajectoryPoint
from VIS_mp_autocoup import AutocoupAnimation

show_animation = True

class Simulation:

    def __init__(self):
            
        self.planner = AutocoupPlanner()
        
        self.init_pose = Pose(None,3,5, np.deg2rad(200),0)
        self.goal_pose = Pose(None,12.25,13.6, np.deg2rad(185),0)
        self.planner.update_ego_pose(self.init_pose)
        self.ego_pose = self.init_pose

        self.animation = AutocoupAnimation()

    def simulate(self):
    
        counter = 0


        while True:


            print("cycle: ", counter)
            counter += 1

            self.ego_pose = self.planner.update_ego_pose_reverse()

            #self.planner.update_ego_pose(self.ego)
            self.planner.update_goal_pose(self.goal_pose)

            self.planner.cycle()
            
            self.animation.update_trajectory_vis(   [tpoint.x for tpoint in self.planner.trajectory],[tpoint.y for tpoint in self.planner.trajectory],\
                                                    [tpoint.x for tpoint in self.planner.trajectory23],[tpoint.y for tpoint in self.planner.trajectory23])

            self.animation.update_pose_vis(self.ego_pose.x,self.ego_pose.y,self.ego_pose.yaw,self.goal_pose.x,self.goal_pose.y,self.goal_pose.yaw)


            self.planner.ego_drive_step()

            time.sleep(0.5)
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()