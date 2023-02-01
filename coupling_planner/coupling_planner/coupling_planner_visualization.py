import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

plt.ion()

class AutocoupAnimation():

    def __init__(self):

        self.xmin = -5
        self.xmax = 5
        self.ymin = -5
        self.ymax = 5

        self.trajectory_p1 = []
        self.trajectory_p2 = []
        self.trajectory23 = []

        self.ego_x = 0
        self.ego_y = 0
        self.ego_yaw = 0
        self.prekingpin_x = 0
        self.prekingpin_y = 0
        self.prekingpin_yaw = 0
        self.kingpin_x = 0
        self.kingpin_y = 0
        self.kingpin_yaw = 0 

        #Set up xy figure
        self.bird_figure, self.bird_axis = plt.subplots()
        self.bird_axis.grid()

        self.trajectory_p1_axis, = self.bird_axis.plot([],[], '-g')
        self.trajectory_p2_axis, = self.bird_axis.plot([],[], '-b')
        self.trajectory23_axis, = self.bird_axis.plot([],[],'-r')

        #setup arrows
        self.arrow1a, = self.bird_axis.plot([],[],'black')
        self.arrow1b, = self.bird_axis.plot([],[],'black')
        self.arrow1c, = self.bird_axis.plot([],[],'black')
        self.arrow2a, = self.bird_axis.plot([],[], '#525252')
        self.arrow2b, = self.bird_axis.plot([],[], '#525252')
        self.arrow2c, = self.bird_axis.plot([],[], '#525252')
        self.arrow3a, = self.bird_axis.plot([],[], '#525252')
        self.arrow3b, = self.bird_axis.plot([],[], '#525252')
        self.arrow3c, = self.bird_axis.plot([],[], '#525252')
        
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
        

    def data_transfer(self,trajectory_p1, trajectory_p2, trajectory23,ego_x,ego_y,ego_yaw,kingpin_x,kingpin_y,kingpin_yaw,prekingpin_x,prekingpin_y,prekingpin_yaw):

        self.trajectory_p1 = trajectory_p1
        self.trajectory_p2 = trajectory_p2
        self.trajectory23 = trajectory23

        self.ego_x = ego_x
        self.ego_y = ego_y
        self.ego_yaw = ego_yaw
        self.prekingpin_x = prekingpin_x
        self.prekingpin_y = prekingpin_y
        self.prekingpin_yaw = prekingpin_yaw
        self.kingpin_x = kingpin_x
        self.kingpin_y = kingpin_y
        self.kingpin_yaw = kingpin_yaw

        self.calc_limits()
        self.update_data_bird_fig()
        self.update_data_graph_fig()
        self.update_data_arrow()
        self.update_figure()

    def calc_limits(self):

        self.xmin = min(self.ego_x, self.kingpin_x, self.prekingpin_x) -2
        self.xmax = max(self.ego_x, self.kingpin_x, self.prekingpin_x) +2   

        self.ymin = min(self.ego_y, self.kingpin_y, self.prekingpin_y) -2
        self.ymax = max(self.ego_y, self.kingpin_y, self.prekingpin_y) +2

    def update_data_bird_fig(self):
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

    def update_data_fancyarrow(self,ego_x,ego_y,ego_yaw,kingpin_x,kinpin_y,kingpin_yaw,prekingpin_x,prekinpin_y,prekingpin_yaw):

        self.ego_arrow.set_data(x=ego_x,y=ego_y, dx=0.5 * np.cos(ego_yaw), dy=0.5 * np.sin(ego_yaw), head_width=0.5, head_length=0.5)
        self.bird_axis.add_patch(self.ego_arrow)

        self.kingpin_arrow.set_data(x=kingpin_x,y=kinpin_y, dx=0.5 * np.cos(kingpin_yaw), dy=0.5 * np.sin(kingpin_yaw), head_width=0.5, head_length=0.5)
        self.bird_axis.add_patch(self.kingpin_arrow)

        self.prekingpin_arrow.set_data(x=prekingpin_x,y=prekinpin_y, dx=0.5 * np.cos(prekingpin_yaw), dy=0.5 * np.sin(prekingpin_yaw), head_width=0.5, head_length=0.5)
        self.bird_axis.add_patch(self.prekingpin_arrow)

    def update_data_arrow(self):

        ego_length = 0.5
        pin_length = 0.35

        ego_Ax = self.ego_x + (ego_length*np.cos(self.ego_yaw))
        ego_Ay = self.ego_y + (ego_length*np.sin(self.ego_yaw))
        ego_Bx = ego_Ax + (ego_length*0.5*np.cos(self.ego_yaw-np.deg2rad(135)))
        ego_By = ego_Ay + (ego_length*0.5*np.sin(self.ego_yaw-np.deg2rad(135)))
        ego_Cx = ego_Ax + (ego_length*0.5*np.cos(self.ego_yaw+np.deg2rad(135)))
        ego_Cy = ego_Ay + (ego_length*0.5*np.sin(self.ego_yaw+np.deg2rad(135)))

        self.arrow1a.set_xdata([self.ego_x,ego_Ax])
        self.arrow1a.set_ydata([self.ego_y,ego_Ay])
        self.arrow1b.set_xdata([ego_Ax,ego_Bx])
        self.arrow1b.set_ydata([ego_Ay,ego_By])
        self.arrow1c.set_xdata([ego_Ax,ego_Cx])
        self.arrow1c.set_ydata([ego_Ay,ego_Cy])

        prekingpin_Ax = self.prekingpin_x + (pin_length*np.cos(self.prekingpin_yaw))
        prekingpin_Ay = self.prekingpin_y + (pin_length*np.sin(self.prekingpin_yaw))
        prekingpin_Bx = prekingpin_Ax + (pin_length*0.5*np.cos(self.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_By = prekingpin_Ay + (pin_length*0.5*np.sin(self.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_Cx = prekingpin_Ax + (pin_length*0.5*np.cos(self.prekingpin_yaw+np.deg2rad(135)))
        prekingpin_Cy = prekingpin_Ay + (pin_length*0.5*np.sin(self.prekingpin_yaw+np.deg2rad(135)))

        self.arrow2a.set_xdata([self.prekingpin_x,prekingpin_Ax])
        self.arrow2a.set_ydata([self.prekingpin_y,prekingpin_Ay])
        self.arrow2b.set_xdata([prekingpin_Ax,prekingpin_Bx])
        self.arrow2b.set_ydata([prekingpin_Ay,prekingpin_By])
        self.arrow2c.set_xdata([prekingpin_Ax,prekingpin_Cx])
        self.arrow2c.set_ydata([prekingpin_Ay,prekingpin_Cy])

        kingpin_Ax = self.kingpin_x + (pin_length*np.cos(self.kingpin_yaw))
        kingpin_Ay = self.kingpin_y + (pin_length*np.sin(self.kingpin_yaw))
        kingpin_Bx = kingpin_Ax + (pin_length*0.5*np.cos(self.kingpin_yaw-np.deg2rad(135)))
        kingpin_By = kingpin_Ay + (pin_length*0.5*np.sin(self.kingpin_yaw-np.deg2rad(135)))
        kingpin_Cx = kingpin_Ax + (pin_length*0.5*np.cos(self.kingpin_yaw+np.deg2rad(135)))
        kingpin_Cy = kingpin_Ay + (pin_length*0.5*np.sin(self.kingpin_yaw+np.deg2rad(135)))

        self.arrow3a.set_xdata([self.kingpin_x,kingpin_Ax])
        self.arrow3a.set_ydata([self.kingpin_y,kingpin_Ay])
        self.arrow3b.set_xdata([kingpin_Ax,kingpin_Bx])
        self.arrow3b.set_ydata([kingpin_Ay,kingpin_By])
        self.arrow3c.set_xdata([kingpin_Ax,kingpin_Cx])
        self.arrow3c.set_ydata([kingpin_Ay,kingpin_Cy])

    def update_figure(self):
        self.bird_axis.set_xlim(self.xmin,self.xmax)
        self.bird_axis.set_ylim(self.ymin,self.ymax)
        self.bird_axis.set_aspect('equal')
        self.bird_figure.canvas.draw()
        self.bird_figure.canvas.flush_events()
        
        for i in range(4):
            self.graph_axis[i].relim()
            self.graph_axis[i].autoscale_view()

        self.graph_figure.canvas.draw()
        self.graph_figure.canvas.flush_events()
        
