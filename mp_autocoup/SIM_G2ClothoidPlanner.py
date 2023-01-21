import time
import matplotlib.pyplot as plt
import numpy as np
from G2ClothoidPlanner import G2ClothoidPlanner, Pose, TrajectoryPoint

show_animation = True

class Simulation:

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self, init_pose, goal_pose,play_area):
        self.planner = G2ClothoidPlanner()
        
        self.init_pose = Pose(init_pose[0],init_pose[1],init_pose[2], init_pose[3])
        self.goal_pose = Pose(goal_pose[0],goal_pose[1],goal_pose[2], goal_pose[3])
        self.planner.update_ego_pose(self.init_pose)
        self.ego = self.init_pose
        

        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
    
    def simulate(self):
    
        counter = 0
        
        while True:

            print("cycle: ", counter)
            counter += 1

            self.ego = self.planner.update_ego_pose_reverse()

            #self.plannermachine.planner.update_ego_pose(self.ego)
            self.planner.update_goal_pose(self.goal_pose)

            self.planner.cycle()
            
            self.visualization()

            self.planner.ego_drive_step()

            time.sleep(0.5)
        
    def visualization(self, rnd = None):
        plt.figure(1)
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])

        
        if self.planner.trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.planner.trajectory:   
                x += [trajectory_point.x]
                y += [trajectory_point.y]
            
            plt.plot(x,y, "-g")
        
        if self.planner.temptrajectory23 is not None:
            
            x = []
            y = []

            for trajectory_point in self.planner.temptrajectory23:   
                x += [trajectory_point.x]
                y += [trajectory_point.y]
            
            plt.plot(x,y, "-r")

        if self.ego is not None:
            self.plot_arrow(self.ego.x,self.ego.y, self.ego.yaw)

        if self.init_pose is not None:
            self.plot_arrow(self.init_pose.x,self.init_pose.y,self.init_pose.yaw,0.75,0.75,"r")

        if self.goal_pose is not None:
            self.plot_arrow(self.goal_pose.x,self.goal_pose.y,self.goal_pose.yaw,0.75,0.75,"r")
            
        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        #plt.axis([-2, 22, -2, 22])
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)

    def LinePlot(self):

        plt.figure(2)
        plt.clf()

        if self.init_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.init_trajectory:   
                x += [trajectory_point.length]
                y += [trajectory_point.curvature]
            

            plt.plot(x,y, "-b")
        
        if self.driven_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.driven_trajectory:   
                x += [trajectory_point.length]
                y += [trajectory_point.curvature]
            

            plt.plot(x,y, "-r")

        if self.planner.trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.planner.trajectory:   
                x += [trajectory_point.length + self.ego.arc_length]
                y += [trajectory_point.curvature]
            
            plt.plot(x,y, "-g")
        
        plt.grid(True)
        
        plt.figure(3)
        plt.clf()

        if self.init_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.init_trajectory:   
                x += [trajectory_point.length]
                y += [trajectory_point.velocity]
            
            plt.plot(x,y, "-g")

            plt.grid(True)
            plt.axis("equal")



    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok
    
    @staticmethod
    def plot_arrow(x, y, yaw, length=0.75, width=0.75, fc="g", ec="k"):  # pragma: no cover
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

    
   
def main():
    print("start " + __file__)
    simulation = Simulation(goal_pose=[12,13, np.deg2rad(90),0],
                            init_pose=[3,5, np.deg2rad(20),0],
                            play_area=[0,20,0,20])

    simulation.simulate()

if __name__ == '__main__':
    main()