import time
import numpy as np

from LIB_mp_autocoup import AutocoupPlanner, Pose, TrajectoryPoint
from VIS_mp_autocoup import AutocoupAnimation

show_animation = True

class Simulation:

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self, init_pose, goal_pose,play_area):
        
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
            
        
        self.planner = AutocoupPlanner()
        
        self.init_pose = Pose(None,init_pose[0],init_pose[1],init_pose[2], init_pose[3])
        self.goal_pose = Pose(None,goal_pose[0],goal_pose[1],goal_pose[2], goal_pose[3])
        self.planner.update_ego_pose(self.init_pose)
        self.ego = self.init_pose

        self.animation = AutocoupAnimation()

    def simulate(self):
    
        counter = 0


        while True:


            print("cycle: ", counter)
            counter += 1

            self.ego = self.planner.update_ego_pose_reverse()

            #self.planner.update_ego_pose(self.ego)
            self.planner.update_goal_pose(self.goal_pose)

            self.planner.cycle()
            
            self.animation.update_trajectory_vis(   [tpoint.x for tpoint in self.planner.trajectory],[tpoint.y for tpoint in self.planner.trajectory],\
                                                    [tpoint.x for tpoint in self.planner.trajectory23],[tpoint.y for tpoint in self.planner.trajectory23])

            #Animation.update_pose_vis(self.ego.x,self.ego.y,self.ego.yaw,self.goal_pose.x,self.goal_pose.y,self.goal_pose.yaw)


            self.planner.ego_drive_step()

            #time.sleep(0.5)

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok
   
def main():
    print("start " + __file__)
    simulation = Simulation(goal_pose=[12,13, np.deg2rad(90),0],
                            init_pose=[3,5, np.deg2rad(20),0],
                            play_area=[0,20,0,20])

    simulation.simulate()

if __name__ == '__main__':
    main()