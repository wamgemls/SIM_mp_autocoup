import time
import numpy as np

from LIB_mp_autocoup import  Pose, TrajectoryPoint, PlannerMode, AutocoupPlanner

class Simulation:

    def __init__(self):
        
        self.planner = AutocoupPlanner()
            
        self.ego_pose = Pose(None, 2, 10, np.deg2rad(190),0.2, 0)
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