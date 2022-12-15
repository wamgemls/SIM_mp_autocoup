import math
import random
import time
import matplotlib.pyplot as plt
import numpy as np

import pyclothoids

show_animation = True

class TrajectoryPoint:

    def __init__(self,length,x=None,y=None,yaw=None,curvature=None,verlocity=None):
        self.length = length
        self.x = x   
        self.y = y
        self.yaw = yaw
        self.curvature = curvature
        self.velocity = verlocity

class Pose:

    def __init__(self, x, y, yaw, curvature):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curvature = curvature

class Ego:

    def __init__(self, pose):
        self.pose = pose
        self.trajectory_list = None
        self.arc_length = 0

    def update_pose(self, trajectory_list):
        self.trajectory_list = trajectory_list
        self.pose = self.trajectory_list[1]

class G2ClothoidPlanner:

        trajectory = []
        trajectory23 = []
        
        def __init__(self, pathres = 0.1):
            self.pathres = pathres

        def generate_trajectory(self, ego_pose, goal_pose):

            g2clothoid_list = pyclothoids.SolveG2(ego_pose.x, ego_pose.y, ego_pose.yaw, ego_pose.curvature, goal_pose.x, goal_pose.y, goal_pose.yaw, goal_pose.curvature)
            g2clothoid_trajectory = self.sample_trajectory_HP(g2clothoid_list)

            self.trajectory = g2clothoid_trajectory
            self.trajectory23 = g2clothoid_trajectory[0:22]
        
            return self.trajectory

        def sample_trajectory(self, g2clothoid_list):

            trajectory_point_list = []
    
            for clothoid in g2clothoid_list:
                trajectory_point_list_temp = [TrajectoryPoint(clothoid.length * m/(round(clothoid.length/self.pathres))) for m in range(0,round(clothoid.length/self.pathres))]

                for trajectory_point in trajectory_point_list_temp:
                    trajectory_point.x = clothoid.X(trajectory_point.length)
                    trajectory_point.y = clothoid.Y(trajectory_point.length)
                    trajectory_point.yaw = clothoid.Theta(trajectory_point.length)
                    trajectory_point.curvature = clothoid.KappaStart + (clothoid.dk*trajectory_point.length)
                    trajectory_point.velocity = 0.5
            
                trajectory_point_list += trajectory_point_list_temp

            return trajectory_point_list

        def sample_trajectory_HP(self, g2clothoid_list):

            total_length = g2clothoid_list[0].length + g2clothoid_list[1].length + g2clothoid_list[2].length
            npt_tar = round(total_length/self.pathres)
            samp_int = total_length/npt_tar

            trajectory_point_list = []
            samp_point = 0
            
            while samp_point < g2clothoid_list[0].length:

                trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                            g2clothoid_list[0].X(samp_point),
                                                            g2clothoid_list[0].Y(samp_point),
                                                            g2clothoid_list[0].Theta(samp_point),
                                                            g2clothoid_list[0].KappaStart + (g2clothoid_list[0].dk*samp_point)))

                samp_point += samp_int

            while samp_point-g2clothoid_list[0].length < g2clothoid_list[1].length:

                trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                            g2clothoid_list[1].X(samp_point - g2clothoid_list[0].length),
                                                            g2clothoid_list[1].Y(samp_point - g2clothoid_list[0].length),
                                                            g2clothoid_list[1].Theta(samp_point - g2clothoid_list[0].length),
                                                            g2clothoid_list[1].KappaStart + (g2clothoid_list[1].dk*(samp_point-g2clothoid_list[0].length))))

                samp_point += samp_int

            while samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length < g2clothoid_list[2].length:
                
                trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                            g2clothoid_list[2].X(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),
                                                            g2clothoid_list[2].Y(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),
                                                            g2clothoid_list[2].Theta(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),
                                                            g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length))))

                samp_point += samp_int


            trajectory_point_list.append(TrajectoryPoint(total_length,
                                                        g2clothoid_list[2].X(g2clothoid_list[2].length),
                                                        g2clothoid_list[2].Y(g2clothoid_list[2].length),
                                                        g2clothoid_list[2].Theta(g2clothoid_list[2].length),
                                                        g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*g2clothoid_list[2].length)))

            return trajectory_point_list

class G1ClothoidPlanner:

    trajectory = []
    trajectory23 = []
    
    def __init__(self):
        pass

    def generate_trajectory(self, ego_pose, goal_pose):

        g2clothoid_list = pyclothoids.Clothoid.G1Hermite(ego_pose.x, ego_pose.y, ego_pose.yaw, goal_pose.x, goal_pose.y, goal_pose.yaw)
        g2clothoid_trajectory = self.sample_trajectory_HP(g2clothoid_list)

        self.trajectory = g2clothoid_trajectory
        self.trajectory23 = g2clothoid_trajectory[0:22]
    
        return self.trajectory

    @staticmethod
    def sample_trajectory(g2clothoid_list):

        trajectory_point_list = []
  
        for clothoid in g2clothoid_list:
            trajectory_point_list_temp = [TrajectoryPoint(clothoid.length * m/(round(clothoid.length/0.1))) for m in range(0,round(clothoid.length/0.1))]

            for trajectory_point in trajectory_point_list_temp:
                trajectory_point.x = clothoid.X(trajectory_point.length)
                trajectory_point.y = clothoid.Y(trajectory_point.length)
                trajectory_point.yaw = clothoid.Theta(trajectory_point.length)
                trajectory_point.curvature = clothoid.KappaStart + (clothoid.dk*trajectory_point.length)
                trajectory_point.velocity = 0.5
        
            trajectory_point_list += trajectory_point_list_temp

        return trajectory_point_list

    @staticmethod
    def sample_trajectory_HP(g1clothoid):

        total_length = g1clothoid.length
        npt_tar = round(total_length/0.1)
        samp_int = total_length/npt_tar

        trajectory_point_list = []
        samp_point = 0
        
        while samp_point < g1clothoid.length:

            trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                        g1clothoid.X(samp_point),
                                                        g1clothoid.Y(samp_point),
                                                        g1clothoid.Theta(samp_point),
                                                        g1clothoid.KappaStart + (g1clothoid.dk*samp_point)))

            samp_point += samp_int

        trajectory_point_list.append(TrajectoryPoint(total_length,
                                                    g1clothoid.X(g1clothoid.length),
                                                    g1clothoid.Y(g1clothoid.length),
                                                    g1clothoid.Theta(g1clothoid.length),
                                                    g1clothoid.KappaStart + (g1clothoid.dk*g1clothoid.length)))

        return trajectory_point_list

        
class Simulation:

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self, init_pose, goal_pose,play_area):
        self.init_pose = Pose(init_pose[0],init_pose[1],init_pose[2], init_pose[3])
        self.goal_pose = Pose(goal_pose[0],goal_pose[1],goal_pose[2], goal_pose[3])
        self.ego = None
        self.planner = None
        self.init_trajectory = []
        self.driven_trajectory = []
        self.perception = None
        self.final_pose_reached = False

        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None

    def simulate(self):

        self.planner = G2ClothoidPlanner()
        self.ego = Ego(self.init_pose)
        self.init_trajectory = self.planner.generate_trajectory(self.init_pose, self.goal_pose)
        counter = 0
        
        while not self.final_pose_reached:

            print("cycle: ", counter)
            print(pyclothoids.__name__)
            counter += 1

            self.planner.generate_trajectory(self.ego.pose, self.goal_pose)

            trajectoryPoint = TrajectoryPoint(  self.planner.trajectory[1].length,
                                                self.planner.trajectory[1].x,
                                                self.planner.trajectory[1].y,
                                                self.planner.trajectory[1].yaw,
                                                self.planner.trajectory[1].curvature)

            self.ego.arc_length += trajectoryPoint.length

            trajectoryPoint.length = self.ego.arc_length
            self.driven_trajectory.append(trajectoryPoint)
            
            self.ego.update_pose(self.planner.trajectory)
            self.visualization()

            if self.ego.pose.x == self.goal_pose.x and self.ego.pose.y == self.goal_pose.y and self.ego.pose.yaw == self.goal_pose.yaw:
                self.final_pose_reached = True
        
        print("Pose reached")
            
    def visualization(self, rnd = None):
        plt.figure(2)
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-o')

        if self.init_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.init_trajectory:   
                x += [trajectory_point.x]
                y += [trajectory_point.y]
            
            plt.plot(x,y, "-b")

        if self.driven_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.driven_trajectory:   
                x += [trajectory_point.x]
                y += [trajectory_point.y]

            plt.plot(x,y, "-r")

        if self.planner.trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.planner.trajectory:   
                x += [trajectory_point.x]
                y += [trajectory_point.y]
            
            plt.plot(x,y, "-g")

        if self.ego.pose is not None:
            self.plot_arrow(self.ego.pose.x,self.ego.pose.y, self.ego.pose.yaw)

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

        plt.axis([-2, 22, -2, 22])
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)

        self.LinePlot()

    def LinePlot(self):

        plt.figure(1)
        plt.clf()
        plt.grid(True)

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
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

    
   
def main():
    print("start " + __file__)
    simulation = Simulation(init_pose=[10,17.5, np.deg2rad(-150),0],
                            goal_pose=[3,5, np.deg2rad(-90),0],
                            play_area=[0,20,0,20])

    simulation.simulate()

    #Test

    print("finished")
    plt.show()

if __name__ == '__main__':
    main()