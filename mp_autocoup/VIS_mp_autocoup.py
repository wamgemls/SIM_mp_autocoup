import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

plt.ion()

class AutocoupAnimation():
    #Suppose we know the x ranges
    min_x = 0
    max_x = 20

    def __init__(self):

        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.trajectory_p1, = self.ax.plot([],[], '-g')
        self.trajectory_p2, = self.ax.plot([],[], '-b')
        self.trajectory23, = self.ax.plot([],[],'-r')

        self.ego_arrow = patches.FancyArrow(0,0,0,0,fc="y", ec="k")
        self.kingpin_arrow = patches.FancyArrow(0,0,0,0,fc="r", ec="k")
        self.prekingpin_arrow = patches.FancyArrow(0,0,0,0,fc="b", ec="k")

        #Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set_ylim(self.min_x,self.max_x)
        self.ax.set_xlim(self.min_x, self.max_x)
        #Other stuff
        self.ax.grid()

    def update_trajectory_vis(self, x_trajectory_p1, y_trajectory_p1,x_trajectory_p2, y_trajectory_p2,x_trajectory23, y_trajectory23):
        #Update data (with the new _and_ the old points)
        self.trajectory_p1.set_xdata(x_trajectory_p1)
        self.trajectory_p1.set_ydata(y_trajectory_p1)

        self.trajectory_p2.set_xdata(x_trajectory_p2)
        self.trajectory_p2.set_ydata(y_trajectory_p2)

        self.trajectory23.set_xdata(x_trajectory23)
        self.trajectory23.set_ydata(y_trajectory23)

        self.update_figure()

    def update_pose_vis(self,ego_x,ego_y,ego_yaw,kingpin_x,kinpin_y,kingpin_yaw,prekingpin_x,prekinpin_y,prekingpin_yaw):

        self.ego_arrow.set_data(x=ego_x,y=ego_y, dx=0.5 * np.cos(ego_yaw), dy=0.5 * np.sin(ego_yaw), head_width=0.5, head_length=0.5)
        self.ax.add_patch(self.ego_arrow)

        self.kingpin_arrow.set_data(x=kingpin_x,y=kinpin_y, dx=0.5 * np.cos(kingpin_yaw), dy=0.5 * np.sin(kingpin_yaw), head_width=0.5, head_length=0.5)
        self.ax.add_patch(self.kingpin_arrow)

        self.prekingpin_arrow.set_data(x=prekingpin_x,y=prekinpin_y, dx=0.5 * np.cos(prekingpin_yaw), dy=0.5 * np.sin(prekingpin_yaw), head_width=0.5, head_length=0.5)
        self.ax.add_patch(self.prekingpin_arrow)
        
        self.update_figure()

    def update_figure(self):
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        #plt.axis("equal")
        #We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
