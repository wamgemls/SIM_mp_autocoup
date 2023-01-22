import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import random

plt.ion()

class AutocoupAnimation():
    #Suppose we know the x range
    min_x = 0
    max_x = 20

    def __init__(self):

        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.trajectory, = self.ax.plot([],[], '-g')
        self.trajectory23, = self.ax.plot([],[],'-r')

        self.ego_pose = patches.FancyArrow(0,0,0,0,fc="r", ec="k")
        self.goal_pose = patches.FancyArrow(0,0,0,0,fc="r", ec="k")

        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set_ylim(self.min_x,self.max_x)
        self.ax.set_xlim(self.min_x, self.max_x)
        #Other stuff
        self.ax.grid()

    def update_trajectory_vis(self, x_trajectory, y_trajectory,x_trajectory23, y_trajectory23):
        #Update data (with the new _and_ the old points)
        self.trajectory.set_xdata(x_trajectory)
        self.trajectory.set_ydata(y_trajectory)

        self.trajectory23.set_xdata(x_trajectory23)
        self.trajectory23.set_ydata(y_trajectory23)

        self.update_figure()

    def update_pose_vis(self,ego_x,ego_y,ego_yaw,goal_x,goal_y,goal_yaw):
        
        self.ego_pose.set_data(x=ego_x, y= ego_y, dx=0.75 * np.cos(ego_yaw), dy=0.75 * np.sin(ego_yaw), head_width=0.75, head_length=0.75)
        self.ax.add_patch(self.ego_pose)

        self.goal_pose.set_data(x=goal_x, y= goal_y, dx=0.75 * np.cos(goal_yaw), dy=0.75 * np.sin(goal_yaw), head_width=0.75, head_length=0.75)
        self.ax.add_patch(self.goal_pose)
        
        self.update_figure()

    def update_figure(self):
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #plt.axis("equal")
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
