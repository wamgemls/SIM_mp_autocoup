import time
import numpy as np

from LIB_mp_autocoup import AutocoupPlanner, Pose, TrajectoryPoint
from VIS_mp_autocoup import AutocoupAnimation

show_animation = True

class Simulation:

    def __init__(self):
        
        self.planner = AutocoupPlanner()
            
        self.init_pose = Pose(None, 10, 17, np.deg2rad(100), 0, 0)
        self.ego_pose = self.init_pose
        self.kingpin_pose = Pose(None, 3, 4, np.deg2rad(40), 0, 0)
        self.prekingpin_pose = Pose()
        self.planner.update_pose(self.init_pose,self.ego_pose,self.kingpin_pose)

        self.animation = AutocoupAnimation()

    def simulate(self):
    
        counter = 0

        while True:

            print("cycle: ",counter,end=' -> ')
            counter += 1

            self.ego_pose,self.prekingpin_pose = self.planner.update_pose_reverse()
            self.planner.update_pose(self.init_pose,self.ego_pose,self.kingpin_pose)

            self.planner.cycle()
            
            self.animation.update_trajectory_vis(   [tpoint.x for tpoint in self.planner.trajectory_p1],[tpoint.y for tpoint in self.planner.trajectory_p1],
                                                    [tpoint.x for tpoint in self.planner.trajectory_p2],[tpoint.y for tpoint in self.planner.trajectory_p2],
                                                    [tpoint.x for tpoint in self.planner.trajectory23],[tpoint.y for tpoint in self.planner.trajectory23])
            
            #self.animation.update_pose_vis( self.ego_pose.x,self.ego_pose.y,self.ego_pose.yaw,\
            #                                   self.kingpin_pose.x,self.kingpin_pose.y,self.kingpin_pose.yaw,\
            #                                       self.prekingpin_pose.x,self.prekingpin_pose.y,self.prekingpin_pose.yaw)

            self.planner.ego_drive_step()

            time.sleep(0.5)
   
def main():
    print("start " + __file__)
    simulation = Simulation()
    simulation.simulate()

if __name__ == '__main__':
    main()