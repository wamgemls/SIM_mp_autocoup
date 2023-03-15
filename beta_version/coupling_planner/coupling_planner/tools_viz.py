import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

plt.ion()

plt.rc('text', usetex=True)
plt.rc('font', family='serif')

class AutocoupAnimation():

    def __init__(self):

        self.xmin = -5
        self.xmax = 5
        self.ymin = -5
        self.ymax = 5

        self.trajectory = []
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

        self.setup_bird_fig()
        self.setup_graph_fig()
        self.setup_graph_fig23()

    def data_transfer(self, trajectory, trajectory23,
                            ego_x,ego_y,ego_yaw,
                            kingpin_x,kingpin_y,kingpin_yaw,
                            prekingpin_x,prekingpin_y,prekingpin_yaw):

        self.trajectory = trajectory
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

    def calc_limits(self):

        self.xmin = min(self.ego_x, self.kingpin_x, self.prekingpin_x)
        self.xmax = max(self.ego_x, self.kingpin_x, self.prekingpin_x)  
        self.ymin = min(self.ego_y, self.kingpin_y, self.prekingpin_y)
        self.ymax = max(self.ego_y, self.kingpin_y, self.prekingpin_y)
        
        p1_xmin = None
        p1_xmax = None
        p1_ymin = None
        p1_ymax = None

        p2_xmin = None
        p2_xmax = None
        p2_ymin = None
        p2_ymax = None

        if self.trajectory:
            for traj_point in self.trajectory:
                if not p1_xmax or p1_xmax < traj_point.x:
                    p1_xmax = traj_point.x
                if not p1_xmin or p1_xmin > traj_point.x:
                    p1_xmin = traj_point.x
                if not p1_ymax or p1_ymax < traj_point.y:
                    p1_ymax = traj_point.y
                if not p1_ymin or p1_ymin > traj_point.y:
                    p1_ymin = traj_point.y
            
            self.xmin = min(self.xmin,p1_xmin)
            self.xmax = max(self.xmax,p1_xmax)
            self.ymin = min(self.ymin,p1_ymin)
            self.ymax = max(self.ymax,p1_ymax)

        self.xmin -= 2
        self.xmax += 2
        self.ymin -= 2
        self.ymax += 2

    def setup_bird_fig(self):

        #Set up bird figure
        self.bird_figure, self.bird_axis = plt.subplots()
        self.bird_axis.grid()

        #setup arrows
        self.ego_arrow_line, = self.bird_axis.plot([],[],'black',zorder=4.1)
        self.prekingpin_line, = self.bird_axis.plot([],[], '#525252',zorder=4.0)
        self.kingpin_line, = self.bird_axis.plot([],[], '#858585',zorder=4.0)

        self.ego_arrow_poly, = self.bird_axis.fill([],[],color="black",zorder=5.1)
        self.prekingpin_arrow_poly, = self.bird_axis.fill([],[],color="#525252",zorder=5.0)
        self.kingpin_arrow_poly, = self.bird_axis.fill([],[],color="#858585",zorder=5.0)
        
        self.trajectory_axis, = self.bird_axis.plot([],[], '-g',zorder=3.0)
        self.trajectory23_axis, = self.bird_axis.plot([],[],'-r',zorder=3.1)
    
    def setup_graph_fig(self):

        #Set up long figure
        self.graph_figure, self.graph_axis = plt.subplots(4,1,sharex=True)
        self.trajectory_p1_vx, = self.graph_axis[0].plot([],[], '-g')
        self.graph_axis[0].set_ylabel(r'$v_x$ $(\frac{m}{s})$',fontsize=12)
        self.graph_axis[3].set_xlabel(r'length $(m)$',fontsize=12)

        self.trajectory_p1_ax, = self.graph_axis[1].plot([],[], '-g')
        self.graph_axis[1].set_ylabel(r'$a_x$ $(\frac{m}{s^2})$',fontsize=12)
        self.graph_axis[3].set_xlabel(r'length $(m)$',fontsize=12)

        self.trajectory_p1_yaw, = self.graph_axis[2].plot([],[], '-g')
        self.graph_axis[2].set_ylabel(r'$\theta$ $(rad)$',fontsize=12)
        self.graph_axis[3].set_xlabel(r'length $(m)$',fontsize=12)

        self.trajectory_p1_curv, = self.graph_axis[3].plot([],[], '-g')
        self.graph_axis[3].set_ylabel(r'$\kappa$ $(\frac{1}{m})$',fontsize=12)
        self.graph_axis[3].set_xlabel(r'length $(m)$',fontsize=12)
        
        for i in range(4):
            self.graph_axis[i].grid()

    def setup_graph_fig23(self):

        #Set up 23 long
        self.graph_figure23, self.graph_axis23 = plt.subplots(4,1,sharex=True)
        self.trajectory_23_vx, = self.graph_axis23[0].plot([],[], '-r')
        self.graph_axis23[0].set_ylabel('vx (m/s)')
        self.graph_axis23[3].set_xlabel('length (m)')
        self.trajectory_23_ax, = self.graph_axis23[1].plot([],[], '-r')
        self.graph_axis23[1].set_ylabel('ax (m/s2)')
        self.graph_axis23[3].set_xlabel('length (m)')
        self.trajectory_23_yaw, = self.graph_axis23[2].plot([],[], '-r')
        self.graph_axis23[2].set_ylabel('yaw (rad)')
        self.graph_axis23[3].set_xlabel('length (m)')
        self.trajectory_23_curv, = self.graph_axis23[3].plot([],[], '-r')
        self.graph_axis23[3].set_ylabel('curvature (1/m)')
        self.graph_axis23[3].set_xlabel('length (m)')

        for i in range(4):
            self.graph_axis23[i].grid()

    def full_update_data_bird_fig(self):
        #Update data (with the new _and_ the old points)
        self.trajectory_axis.set_xdata([tpoint.x for tpoint in self.trajectory])
        self.trajectory_axis.set_ydata([tpoint.y for tpoint in self.trajectory])
        
        self.trajectory23_axis.set_xdata([tpoint.x for tpoint in self.trajectory23])
        self.trajectory23_axis.set_ydata([tpoint.y for tpoint in self.trajectory23])

    def lean_update_data_bird_fig(self):
        self.trajectory23_axis.set_xdata([tpoint.x for tpoint in self.trajectory23])
        self.trajectory23_axis.set_ydata([tpoint.y for tpoint in self.trajectory23])

    def full_update_data_graph_fig(self):
        #Update data (with the new _and_ the old points)
        self.trajectory_p1_vx.set_xdata([tpoint.s for tpoint in self.trajectory])
        self.trajectory_p1_vx.set_ydata([tpoint.vx for tpoint in self.trajectory])

        self.trajectory_p1_ax.set_xdata([tpoint.s for tpoint in self.trajectory])
        self.trajectory_p1_ax.set_ydata([tpoint.ax for tpoint in self.trajectory])

        self.trajectory_p1_yaw.set_xdata([tpoint.s for tpoint in self.trajectory])
        self.trajectory_p1_yaw.set_ydata([tpoint.yaw for tpoint in self.trajectory])

        self.trajectory_p1_curv.set_xdata([tpoint.s for tpoint in self.trajectory])
        self.trajectory_p1_curv.set_ydata([tpoint.curvature for tpoint in self.trajectory])

    def full_update_data_graph_fig23(self):
        #Update data (with the new _and_ the old points)
        self.trajectory_23_vx.set_xdata([tpoint.s for tpoint in self.trajectory23])
        self.trajectory_23_vx.set_ydata([tpoint.vx for tpoint in self.trajectory23])

        self.trajectory_23_ax.set_xdata([tpoint.s for tpoint in self.trajectory23])
        self.trajectory_23_ax.set_ydata([tpoint.ax for tpoint in self.trajectory23])

        self.trajectory_23_yaw.set_xdata([tpoint.s for tpoint in self.trajectory23])
        self.trajectory_23_yaw.set_ydata([tpoint.yaw for tpoint in self.trajectory23])

        self.trajectory_23_curv.set_xdata([tpoint.s for tpoint in self.trajectory23])
        self.trajectory_23_curv.set_ydata([tpoint.curvature for tpoint in self.trajectory23])

    def full_update_data_arrow(self):

        ego_length = 0.5
        pin_length = 0.35

        ego_Ax = self.ego_x + (ego_length*np.cos(self.ego_yaw))
        ego_Ay = self.ego_y + (ego_length*np.sin(self.ego_yaw))
        ego_Bx = ego_Ax + (ego_length*0.5*np.cos(self.ego_yaw-np.deg2rad(150)))
        ego_By = ego_Ay + (ego_length*0.5*np.sin(self.ego_yaw-np.deg2rad(150)))
        ego_Cx = ego_Ax + (ego_length*0.5*np.cos(self.ego_yaw+np.deg2rad(150)))
        ego_Cy = ego_Ay + (ego_length*0.5*np.sin(self.ego_yaw+np.deg2rad(150)))

        self.ego_arrow_line.set_xdata([self.ego_x,ego_Ax])
        self.ego_arrow_line.set_ydata([self.ego_y,ego_Ay])
        self.ego_arrow_poly.set_xy([(ego_Ax,ego_Ay),
                                    (ego_Bx,ego_By),
                                    (ego_Cx,ego_Cy)])

        prekingpin_Ax = self.prekingpin_x + (pin_length*np.cos(self.prekingpin_yaw))
        prekingpin_Ay = self.prekingpin_y + (pin_length*np.sin(self.prekingpin_yaw))
        prekingpin_Bx = prekingpin_Ax + (pin_length*0.5*np.cos(self.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_By = prekingpin_Ay + (pin_length*0.5*np.sin(self.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_Cx = prekingpin_Ax + (pin_length*0.5*np.cos(self.prekingpin_yaw+np.deg2rad(135)))
        prekingpin_Cy = prekingpin_Ay + (pin_length*0.5*np.sin(self.prekingpin_yaw+np.deg2rad(135)))

        self.prekingpin_line.set_xdata([self.prekingpin_x,prekingpin_Ax])
        self.prekingpin_line.set_ydata([self.prekingpin_y,prekingpin_Ay])
        self.prekingpin_arrow_poly.set_xy([ (prekingpin_Ax,prekingpin_Ay),
                                            (prekingpin_Bx,prekingpin_By),
                                            (prekingpin_Cx,prekingpin_Cy)])


        kingpin_Ax = self.kingpin_x + (pin_length*np.cos(self.kingpin_yaw))
        kingpin_Ay = self.kingpin_y + (pin_length*np.sin(self.kingpin_yaw))
        kingpin_Bx = kingpin_Ax + (pin_length*0.5*np.cos(self.kingpin_yaw-np.deg2rad(135)))
        kingpin_By = kingpin_Ay + (pin_length*0.5*np.sin(self.kingpin_yaw-np.deg2rad(135)))
        kingpin_Cx = kingpin_Ax + (pin_length*0.5*np.cos(self.kingpin_yaw+np.deg2rad(135)))
        kingpin_Cy = kingpin_Ay + (pin_length*0.5*np.sin(self.kingpin_yaw+np.deg2rad(135)))

        self.kingpin_line.set_xdata([self.kingpin_x,kingpin_Ax])
        self.kingpin_line.set_ydata([self.kingpin_y,kingpin_Ay])
        self.kingpin_arrow_poly.set_xy([(kingpin_Ax,kingpin_Ay),
                                        (kingpin_Bx,kingpin_By),
                                        (kingpin_Cx,kingpin_Cy)])

    def lean_draw(self):
        self.calc_limits()

        self.bird_axis.set_xlim(self.xmin,self.xmax)
        self.bird_axis.set_ylim(self.ymin,self.ymax)
        self.bird_axis.set_aspect('equal')
        #print(self.bird_axis.dataLim)
        
        self.bird_figure.canvas.draw()
        self.bird_figure.canvas.flush_events()

    def full_draw(self):

        self.calc_limits()
        self.full_update_data_arrow()
        self.full_update_data_bird_fig()
        self.full_update_data_graph_fig()
        self.full_update_data_graph_fig23()
        

        self.bird_axis.set_xlim(self.xmin,self.xmax)
        self.bird_axis.set_ylim(self.ymin,self.ymax)
        self.bird_axis.set_aspect('equal')
        #print(self.bird_axis.dataLim)
        
        self.bird_figure.canvas.draw()
        self.bird_figure.canvas.flush_events()
        
        for i in range(4):
            self.graph_axis[i].relim()
            self.graph_axis[i].autoscale_view()


        self.graph_figure.canvas.draw()
        self.graph_figure.canvas.flush_events()
        self.graph_figure.savefig('velocityprofile', dpi=300)

        for i in range(4):
            self.graph_axis23[i].relim()
            self.graph_axis23[i].autoscale_view()

        self.graph_figure23.canvas.draw()
        self.graph_figure23.canvas.flush_events()
        
