import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

plt.ion()

class AutocoupAnimation():

    def __init__(self):

        self.trajectory_p1 = []
        self.trajectory_p2 = []
        self.trajectory23 = []

        #Set up xy figure
        self.bird_figure, self.bird_axis = plt.subplots()
        self.bird_axis.grid()

        self.trajectory_p1_axis, = self.bird_axis.plot([],[], '-g')
        self.trajectory_p2_axis, = self.bird_axis.plot([],[], '-b')
        self.trajectory23_axis, = self.bird_axis.plot([],[],'-r')

        self.ego_arrow = patches.FancyArrow(0,0,0,0,fc="y", ec="k")
        self.kingpin_arrow = patches.FancyArrow(0,0,0,0,fc="r", ec="k")
        self.prekingpin_arrow = patches.FancyArrow(0,0,0,0,fc="b", ec="k")

        #Set up long figure
        self.graph_figure, self.graph_axis = plt.subplots(4,1,sharex=True)
        self.trajectory_p1_vx, = self.graph_axis[0].plot([],[], '-g')
        self.trajectory_p2_vx, = self.graph_axis[0].plot([],[], '-b')
        self.graph_axis[0].set_ylabel('vx (m/s)')
        self.graph_axis[3].set_xlabel('length (m)')

        self.trajectory_p1_ax, = self.graph_axis[1].plot([],[], '-g')
        self.trajectory_p2_ax, = self.graph_axis[1].plot([],[], '-b')
        self.graph_axis[1].set_ylabel('ax (m/s2)')
        self.graph_axis[3].set_xlabel('length (m)')

        self.trajectory_p1_yaw, = self.graph_axis[2].plot([],[], '-g')
        self.trajectory_p2_yaw, = self.graph_axis[2].plot([],[], '-b')
        self.graph_axis[2].set_ylabel('yaw (rad)')
        self.graph_axis[3].set_xlabel('length (m)')

        self.trajectory_p1_curv, = self.graph_axis[3].plot([],[], '-g')
        self.trajectory_p2_curv, = self.graph_axis[3].plot([],[], '-b')
        self.graph_axis[3].set_ylabel('curvature (1/m)')
        self.graph_axis[3].set_xlabel('length (m)')
        
        for i in range(4):
            self.graph_axis[i].grid()
        
    def data_transfer(self,trajectory_p1, trajectory_p2, trajectory23):

        self.trajectory_p1 = trajectory_p1
        self.trajectory_p2 = trajectory_p2
        self.trajectory23 = trajectory23

        self.update_data_xy_fig()
        self.update_data_graph_fig()
        self.update_figure()

    def update_data_xy_fig(self):
        #Update data (with the new _and_ the old points)
        self.trajectory_p1_axis.set_xdata([tpoint.x for tpoint in self.trajectory_p1])
        self.trajectory_p1_axis.set_ydata([tpoint.y for tpoint in self.trajectory_p1])

        self.trajectory_p2_axis.set_xdata([tpoint.x for tpoint in self.trajectory_p2])
        self.trajectory_p2_axis.set_ydata([tpoint.y for tpoint in self.trajectory_p2])

        self.trajectory23_axis.set_xdata([tpoint.x for tpoint in self.trajectory23])
        self.trajectory23_axis.set_ydata([tpoint.y for tpoint in self.trajectory23])

    def update_data_graph_fig(self):
        #Update data (with the new _and_ the old points)
        self.trajectory_p1_vx.set_xdata([tpoint.s for tpoint in self.trajectory_p1])
        self.trajectory_p1_vx.set_ydata([tpoint.vx for tpoint in self.trajectory_p1])
        self.trajectory_p2_vx.set_xdata([tpoint.s + self.trajectory_p1[-1].s for tpoint in self.trajectory_p2])
        self.trajectory_p2_vx.set_ydata([tpoint.vx for tpoint in self.trajectory_p2])

        self.trajectory_p1_ax.set_xdata([tpoint.s for tpoint in self.trajectory_p1])
        self.trajectory_p1_ax.set_ydata([tpoint.ax for tpoint in self.trajectory_p1])
        self.trajectory_p2_ax.set_xdata([tpoint.s + self.trajectory_p1[-1].s for tpoint in self.trajectory_p2])
        self.trajectory_p2_ax.set_ydata([tpoint.ax for tpoint in self.trajectory_p2])

        self.trajectory_p1_yaw.set_xdata([tpoint.s for tpoint in self.trajectory_p1])
        self.trajectory_p1_yaw.set_ydata([tpoint.yaw for tpoint in self.trajectory_p1])
        self.trajectory_p2_yaw.set_xdata([tpoint.s + self.trajectory_p1[-1].s for tpoint in self.trajectory_p2])
        self.trajectory_p2_yaw.set_ydata([tpoint.yaw for tpoint in self.trajectory_p2])

        self.trajectory_p1_curv.set_xdata([tpoint.s for tpoint in self.trajectory_p1])
        self.trajectory_p1_curv.set_ydata([tpoint.curvature for tpoint in self.trajectory_p1])
        self.trajectory_p2_curv.set_xdata([tpoint.s + self.trajectory_p1[-1].s for tpoint in self.trajectory_p2])
        self.trajectory_p2_curv.set_ydata([tpoint.curvature for tpoint in self.trajectory_p2])

    def update_data_pose(self,ego_x,ego_y,ego_yaw,kingpin_x,kinpin_y,kingpin_yaw,prekingpin_x,prekinpin_y,prekingpin_yaw):

        self.ego_arrow.set_data(x=ego_x,y=ego_y, dx=0.5 * np.cos(ego_yaw), dy=0.5 * np.sin(ego_yaw), head_width=0.5, head_length=0.5)
        self.bird_axis.add_patch(self.ego_arrow)

        self.kingpin_arrow.set_data(x=kingpin_x,y=kinpin_y, dx=0.5 * np.cos(kingpin_yaw), dy=0.5 * np.sin(kingpin_yaw), head_width=0.5, head_length=0.5)
        self.bird_axis.add_patch(self.kingpin_arrow)

        self.prekingpin_arrow.set_data(x=prekingpin_x,y=prekinpin_y, dx=0.5 * np.cos(prekingpin_yaw), dy=0.5 * np.sin(prekingpin_yaw), head_width=0.5, head_length=0.5)
        self.bird_axis.add_patch(self.prekingpin_arrow)
        
        self.update_figure()

    def update_figure(self):
        self.bird_axis.relim()
        self.bird_axis.set_aspect('equal', 'datalim')

        self.bird_figure.canvas.draw()
        self.bird_figure.canvas.flush_events()

        for i in range(4):
            self.graph_axis[i].relim()
            self.graph_axis[i].autoscale_view()

        self.graph_figure.canvas.draw()
        self.graph_figure.canvas.flush_events()
