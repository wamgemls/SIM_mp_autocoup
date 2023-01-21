import matplotlib.pyplot as plt

import numpy as np
import pyclothoids as pyclothoid

import math

init_pose=[5, 10, np.deg2rad(-90),0]
goal_pose=[15,18, np.deg2rad(80),0]

class TrajectoryPoint:

    def __init__(self, length):
        self.length = length
        self.x = None   
        self.y = None 
        self.yaw = None
        self.curvature = None
        self.velocity = None
        self.acceleration = None

def sample_g2clothoid_trajectory(g2clothoid_list):

    trajectory_point_list = []

    for clothoid in g2clothoid_list:
        trajectory_point_list_temp = [TrajectoryPoint(clothoid.length * m/(round(clothoid.length/0.01)-1)) for m in range(0,round(clothoid.length/0.01))]

        for trajectory_point in trajectory_point_list_temp:
            trajectory_point.x = clothoid.X(trajectory_point.length)
            trajectory_point.y = clothoid.Y(trajectory_point.length)
            trajectory_point.yaw = clothoid.Theta(trajectory_point.length)
            trajectory_point.curvature = clothoid.KappaStart + (clothoid.dk*trajectory_point.length)
    
        trajectory_point_list += trajectory_point_list_temp

    return trajectory_point_list

def draw_graph(clothoid_listG2=None, clothoid_list_2 = None):
    plt.clf()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
    



    
    if clothoid_listG2 is not None:
    
        x = []
        y = []

        for trajectory_point in clothoid_listG2:   
            x += [trajectory_point.x]
            y += [trajectory_point.y]
        
        plt.plot(x,y, "-g")

    if clothoid_list_2 is not None:

        x_a = []
        y_a = []

        for trajectory_point in clothoid_list_2:   
            x_a += [trajectory_point.x]
            y_a += [trajectory_point.y]
            
        plt.plot(x_a,y_a, "-r")

    
    
    

    plot_arrow(init_pose[0],init_pose[1],init_pose[2])
    plot_arrow(goal_pose[0],goal_pose[1],goal_pose[2])

    plt.axis([0, 20, 0, 20])
    plt.axis("equal")
    plt.legend(loc='lower left', ncol=2)
    plt.grid(True)
    plt.pause(0.01)

def plot_arrow(x, y, yaw, length=0.75, width=0.75, fc="g", ec="k"):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)

def main():
    print("start " + __file__)

    g2clothoid_list = pyclothoid.SolveG2(init_pose[0], init_pose[1], init_pose[2], init_pose[3], goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3])

    g2clothoid = sample_g2clothoid_trajectory(g2clothoid_list)

    testpoint = g2clothoid[200]

    g2clothoid_list_test = pyclothoid.SolveG2(testpoint.x, testpoint.y, testpoint.yaw, testpoint.curvature, goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3])
    
    g2clothoid_list_test = sample_g2clothoid_trajectory(g2clothoid_list_test)

    draw_graph(g2clothoid, g2clothoid_list_test)
        

    plt.show()

if __name__ == '__main__':
    main()


print("done")