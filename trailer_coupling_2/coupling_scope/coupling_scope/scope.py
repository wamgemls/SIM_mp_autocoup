import matplotlib.pyplot as plt
import numpy as np
from PyQt5 import QtWidgets, QtGui
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtCore import QTimer
import csv
import os

class PlannerData():

    def __init__(self):

        self.xmin = -5
        self.xmax = 5
        self.ymin = -5
        self.ymax = 5

        self.trajectory = []
        self.trajectory23 = []

        self.ego_x = 0
        self.ego_y = 0
        self.ego_yaw = 0
        self.ego_vx = 0
        self.ego_curvature = 0

        self.prekingpin_x = 0
        self.prekingpin_y = 0
        self.prekingpin_yaw = 0

        self.kingpin_x = 0
        self.kingpin_y = 0
        self.kingpin_yaw = 0

        self.dis_error_trajectory = 0
        self.yaw_error_trajectory = 0

        self.dis_error_trajectory_goal = 0
        self.yaw_error_trajectory_goal = 0

        self.dis_error_prekingpin = 0
        self.yaw_error_prekingpin = 0
        self.dis_error_kingpin = 0
        self.yaw_error_kingpin = 0
    
    def data2csv(self, timestamp,csv_path,csv_filename):

        if not os.path.exists(csv_path + csv_filename + ".csv"):
            with open(csv_path + csv_filename + ".csv", mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['timestamp', 
                                    'ego_x', 'ego_y','ego_yaw', 'ego_vx', 'ego_curvature', 
                                        'prekingpin_x', 'prekingpin_y', 'prekingpin_yaw', 
                                            'kingpin_x', 'kingpin_y', 'kingpin_yaw',
                                                'dis_error_trajectory','yaw_error_trajectory', 
                                                'dis_error_trajectory_goal', 'yaw_error_trajectory_goal',
                                                    'dis_error_prekingpin', 'yaw_error_prekingpin', 
                                                        'dis_error_kingpin','yaw_error_kingpin']+['|']+['trajectory']+['|']+['trajectory23'])
        else:
            if timestamp != 0.0:
                with open(csv_path + csv_filename + ".csv",mode='a',newline='') as file:
                    writer = csv.writer(file)

                    trajectory_csv = []
                    for point in self.trajectory:
                        trajectory_csv.append(point.t)
                        trajectory_csv.append(point.s)
                        trajectory_csv.append(point.x)
                        trajectory_csv.append(point.y)
                        trajectory_csv.append(point.vx)
                        trajectory_csv.append(point.ax)
                        trajectory_csv.append(point.yaw)
                        trajectory_csv.append(point.curvature)

                    trajectory23_csv = []
                    for point in self.trajectory23:
                        trajectory23_csv.append(point.t)
                        trajectory23_csv.append(point.s)
                        trajectory23_csv.append(point.x)
                        trajectory23_csv.append(point.y)
                        trajectory23_csv.append(point.vx)
                        trajectory23_csv.append(point.ax)
                        trajectory23_csv.append(point.yaw)
                        trajectory23_csv.append(point.curvature)

                    writer.writerow([timestamp,
                                        self.ego_x,self.ego_y,self.ego_yaw,self.ego_vx,self.ego_curvature,
                                            self.prekingpin_x,self.prekingpin_y,self.prekingpin_yaw,
                                                self.kingpin_x,self.kingpin_y,self.kingpin_yaw,
                                                    self.dis_error_trajectory,self.yaw_error_trajectory,
                                                        self.dis_error_trajectory_goal,self.yaw_error_trajectory_goal,
                                                            self.dis_error_prekingpin,self.yaw_error_prekingpin,
                                                                self.dis_error_kingpin,self.yaw_error_kingpin]+["|"]+trajectory23_csv+["|"]+trajectory_csv)

    def data_transfer(self, trajectory, trajectory23,
                            ego_x,ego_y,ego_yaw,ego_vx,ego_curvature,
                            kingpin_x,kingpin_y,kingpin_yaw,
                            prekingpin_x,prekingpin_y,prekingpin_yaw,
                            ):

        self.trajectory = trajectory
        self.trajectory23 = trajectory23

        self.ego_x = ego_x
        self.ego_y = ego_y
        self.ego_yaw = ego_yaw
        self.ego_vx = ego_vx
        self.ego_curvature = ego_curvature
        self.kingpin_x = kingpin_x
        self.kingpin_y = kingpin_y
        self.kingpin_yaw = kingpin_yaw
        self.prekingpin_x = prekingpin_x
        self.prekingpin_y = prekingpin_y
        self.prekingpin_yaw = prekingpin_yaw

        if self.trajectory:  
            closest_trajpoint = self.trajectory[0]
            for trajpoint in self.trajectory:                                            
                if self.calc_distance_angle_PoseA_PoseB(trajpoint.x,trajpoint.y,trajpoint.yaw,ego_x,ego_y,ego_yaw)[0] < \
                        self.calc_distance_angle_PoseA_PoseB(closest_trajpoint.x,closest_trajpoint.y,closest_trajpoint.yaw,ego_x,ego_y,ego_yaw)[0]:
                    closest_trajpoint = trajpoint
            
            self.dis_error_trajectory, self.yaw_error_trajectory = self.calc_distance_angle_PoseA_PoseB( 
                                                                            closest_trajpoint.x,closest_trajpoint.y,closest_trajpoint.yaw,
                                                                            self.ego_x,self.ego_y,self.ego_yaw)
        else:
            self.dis_error_trajectory = np.inf
            self.yaw_error_trajectory = np.inf

        if self.trajectory:
            self.dis_error_trajectory_goal, self.yaw_error_trajectory_goal = self.calc_distance_angle_PoseA_PoseB( 
                                                                                    self.trajectory[-1].x,self.trajectory[-1].y,self.trajectory[-1].yaw,
                                                                                    self.ego_x,self.ego_y,self.ego_yaw)
        else:
            self.dis_error_trajectory_goal = np.inf
            self.yaw_error_trajectory_goal = np.inf

        self.dis_error_prekingpin, self.yaw_error_prekingpin = self.calc_distance_angle_PoseA_PoseB( 
                                                                                    self.prekingpin_x,prekingpin_y,self.prekingpin_yaw,
                                                                                    self.ego_x,self.ego_y,self.ego_yaw)
        
        self.dis_error_kingpin, self.yaw_error_kingpin = self.calc_distance_angle_PoseA_PoseB( 
                                                                                    self.kingpin_x,kingpin_y,self.kingpin_yaw,
                                                                                    self.ego_x,self.ego_y,self.ego_yaw)
                                                  
    def calc_limits(self):

        self.xmin = min(self.ego_x, self.kingpin_x,self.prekingpin_x)
        self.xmax = max(self.ego_x, self.kingpin_x,self.prekingpin_x)  
        self.ymin = min(self.ego_y, self.kingpin_y,self.prekingpin_y)
        self.ymax = max(self.ego_y, self.kingpin_y,self.prekingpin_y)
        
        p1_xmin = None
        p1_xmax = None
        p1_ymin = None
        p1_ymax = None

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

    def calc_distance_angle_PoseA_PoseB(self,PoseA_x,PoseA_y,PoseA_yaw,PoseB_x,PoseB_y,PoseB_yaw):
        dx = PoseA_x - PoseB_x
        dy = PoseA_y - PoseB_y
        d = np.hypot(dx, dy)
        theta = self.angle_interval(PoseB_yaw-PoseA_yaw)
        return d, theta
    
    @staticmethod
    def angle_interval(angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

class StatusScreen(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.hbox = QtWidgets.QHBoxLayout(self)
        self.vbox_left = QtWidgets.QVBoxLayout()

        self.bird_figure, self.bird_axis = plt.subplots()
        self.bird_axis.grid()
        self.bird_figure_canvas = FigureCanvas(self.bird_figure)

        self.vbox_left.addWidget(self.bird_figure_canvas)
        self.hbox.addLayout(self.vbox_left,2)

        #setup arrows
        self.ego_arrow_line, = self.bird_axis.plot([],[],'black',zorder=5.0)
        self.kingpin_line, = self.bird_axis.plot([],[], '#858585',zorder=4.0)
        self.prekingpin_line, = self.bird_axis.plot([],[], '#585858',zorder=4.0)

        self.ego_arrow_poly, = self.bird_axis.fill([],[],color="black",zorder=5.1)
        self.kingpin_arrow_poly, = self.bird_axis.fill([],[],color="#858585",zorder=4.1)
        self.prekingpin_arrow_poly, = self.bird_axis.fill([],[],color="#585858",zorder=4.1)
        
        self.trajectory_axis, = self.bird_axis.plot([],[], '-g',zorder=3.0)
        self.trajectory23_axis, = self.bird_axis.plot([],[],'-r',zorder=3.1)

        #StatusScreen values
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_pose_label = QtWidgets.QLabel(str("x: ")+str(self.ego_x)+str(" y: ")+str(self.ego_y)+str(" yaw: ")+str(self.ego_yaw))
        self.ego_vx = 0.0
        self.ego_vx_label = QtWidgets.QLabel(str(self.ego_vx))
        self.ego_curvature = 0.0
        self.ego_curvature_label = QtWidgets.QLabel(str(self.ego_curvature))

        self.prekingpin_x = 0.0
        self.prekingpin_y = 0.0
        self.prekingpin_yaw = 0.0
        self.prekingpin_pose_label = QtWidgets.QLabel(str("x: ")+str(self.prekingpin_x)+str(" y: ")+str(self.prekingpin_y)+str(" yaw: ")+str(self.prekingpin_yaw))
        
        self.kingpin_x = 0.0
        self.kingpin_y = 0.0
        self.kingpin_yaw = 0.0
        self.kingpin_pose_label = QtWidgets.QLabel(str("x: ")+str(self.kingpin_x)+str(" y: ")+str(self.kingpin_y)+str(" yaw: ")+str(self.kingpin_yaw))

        self.dis_error_trajectory = 0
        self.dis_error_trajectory_label = QtWidgets.QLabel(str(self.dis_error_trajectory))
        self.yaw_error_trajectory = 0
        self.yaw_error_trajectory_label = QtWidgets.QLabel(str(self.yaw_error_trajectory))

        self.dis_error_trajectory_goal = 0
        self.dis_error_trajectory_goal_label = QtWidgets.QLabel(str(self.dis_error_trajectory_goal))
        self.yaw_error_trajectory_goal = 0
        self.yaw_error_trajectory_goal_label = QtWidgets.QLabel(str(self.yaw_error_trajectory_goal))

        self.dis_error_prekingpin = 0
        self.dis_error_prekingpin_label = QtWidgets.QLabel(str(self.dis_error_prekingpin))
        self.yaw_error_prekingpin = 0
        self.yaw_error_prekingpin_label = QtWidgets.QLabel(str(self.yaw_error_prekingpin))

        self.dis_error_kingpin = 0
        self.dis_error_kingpin_label = QtWidgets.QLabel(str(self.dis_error_kingpin))
        self.yaw_error_kingpin = 0
        self.yaw_error_kingpin_label = QtWidgets.QLabel(str(self.yaw_error_kingpin))

        #GoalInput values
        self.send_goal = False
        self.send_goal_label = QtWidgets.QLabel(str(self.send_goal))

        self.goal_x_input = QtWidgets.QLineEdit(self)
        self.goal_y_input = QtWidgets.QLineEdit(self)
        self.goal_yaw_input = QtWidgets.QLineEdit(self)

        self.btn_send_goal = QtWidgets.QPushButton("send",self)
        self.btn_send_goal.clicked.connect(self.btnListener_start_send_goal)
        self.timer = QTimer(self)
        self.timer.setInterval(500) 
        self.timer.timeout.connect(self.timer_reset) 

        
        # Add a widget to display the status screen

        self.info_layout = QtWidgets.QVBoxLayout()
        self.title_layout = QtWidgets.QVBoxLayout()
        status_screen_label = QtWidgets.QLabel("StatusScreen")
        status_screen_label.setStyleSheet("font-weight: bold")
        self.title_layout.addWidget(status_screen_label)
        self.info_layout.addLayout(self.title_layout)
        
        self.status_ego_layout = QtWidgets.QVBoxLayout()
        self.status_ego_form = QtWidgets.QFormLayout()
        self.status_ego_form.addRow(QtWidgets.QLabel("ego_pose:"),self.ego_pose_label)
        self.status_ego_form.addRow(QtWidgets.QLabel("ego_vx:"),self.ego_vx_label)
        self.status_ego_form.addRow(QtWidgets.QLabel("ego_curvature:"),self.ego_curvature_label)
        self.status_ego_layout.addLayout(self.status_ego_form)
        self.info_layout.addLayout(self.status_ego_layout)

        self.status_goal_layout = QtWidgets.QVBoxLayout()
        self.status_goal_form = QtWidgets.QFormLayout()
        self.status_goal_form.addRow(QtWidgets.QLabel("prekingpin_pose:"),self.prekingpin_pose_label)
        self.status_goal_form.addRow(QtWidgets.QLabel("kingpin_pose:"),self.kingpin_pose_label)
        self.status_goal_layout.addLayout(self.status_goal_form)
        self.info_layout.addLayout(self.status_goal_layout)

        self.status_error_layout = QtWidgets.QVBoxLayout()
        self.status_error_form = QtWidgets.QFormLayout()
        self.status_error_form.addRow(QtWidgets.QLabel("dis_error_trajectory"),self.dis_error_trajectory_label)
        self.status_error_form.addRow(QtWidgets.QLabel("yaw_error_trajectory"),self.yaw_error_trajectory_label)
        self.status_error_form.addRow(QtWidgets.QLabel("dis_error_trajectory_goal"),self.dis_error_trajectory_goal_label)
        self.status_error_form.addRow(QtWidgets.QLabel("yaw_error_trajectory_goal"),self.yaw_error_trajectory_goal_label)
        self.status_error_form.addRow(QtWidgets.QLabel("dis_error_prekingpin"),self.dis_error_prekingpin_label)
        self.status_error_form.addRow(QtWidgets.QLabel("yaw_error_prekingpin"),self.yaw_error_prekingpin_label)
        self.status_error_form.addRow(QtWidgets.QLabel("dis_error_kingpin"),self.dis_error_kingpin_label)
        self.status_error_form.addRow(QtWidgets.QLabel("yaw_error_kingpin"),self.yaw_error_kingpin_label)
        self.status_error_layout.addLayout(self.status_error_form)
        self.info_layout.addLayout(self.status_error_layout)
        
        self.goal_layout = QtWidgets.QFormLayout()
        self.goal_layout.addRow(QtWidgets.QLabel("GoalInput - truck_base"))
        self.goal_layout.addRow(QtWidgets.QLabel("goal_x (m):"),self.goal_x_input)
        self.goal_layout.addRow(QtWidgets.QLabel("goal_y (m):"),self.goal_y_input)
        self.goal_layout.addRow(QtWidgets.QLabel("goal_yaw (°):"),self.goal_yaw_input)
        self.goal_layout.addRow(QtWidgets.QLabel("send_goal"),self.send_goal_label)
        self.goal_layout.addRow(self.btn_send_goal)

        self.info_layout.addLayout(self.goal_layout)

        self.hbox.addLayout(self.info_layout,1)
    
    def btnListener_start_send_goal(self):
        self.send_goal = True
        self.send_goal_label.setText(str(self.send_goal))
        self.timer.start()

    def timer_reset(self):
        self.send_goal = False
        self.send_goal_label.setText(str(self.send_goal))
        self.timer.stop()

    def labelupdate(self,   ego_x,ego_y,ego_yaw,ego_vx,ego_curvature,
                            prekingpin_x,prekingpin_y,prekingpin_yaw,
                            kingpin_x,kingpin_y,kingpin_yaw,
                            dis_error_trajectory,yaw_error_trajectory,
                            dis_error_trajectory_goal,yaw_error_trajectory_goal,
                            dis_error_prekingpin,yaw_error_prekingpin,
                            dis_error_kingpin,yaw_error_kingpin
                        ):

        self.ego_x = ego_x
        self.ego_y = ego_y
        self.ego_yaw = ego_yaw
        self.ego_pose_label.setText(str("x: ")+str(self.ego_x)+str(" y: ")+str(self.ego_y)+str(" yaw: ")+str(self.ego_yaw))
        self.ego_vx = ego_vx
        self.ego_vx_label.setText(str(self.ego_vx))
        self.ego_curvature = ego_curvature
        self.ego_curvature_label.setText(str(self.ego_curvature))

        self.prekingpin_x = prekingpin_x
        self.prekingpin_y = prekingpin_y
        self.prekingpin_yaw = prekingpin_yaw
        self.prekingpin_pose_label.setText(str("x: ")+str(self.prekingpin_x)+str(" y: ")+str(self.prekingpin_y)+str(" yaw: ")+str(self.prekingpin_yaw))

        self.kingpin_x = kingpin_x
        self.kingpin_y = kingpin_y
        self.kingpin_yaw = kingpin_yaw
        self.kingpin_pose_label.setText(str("x: ")+str(self.kingpin_x)+str(" y: ")+str(self.kingpin_y)+str(" yaw: ")+str(self.kingpin_yaw))

        self.dis_error_trajectory = dis_error_trajectory
        self.dis_error_trajectory_label.setText(str(self.dis_error_trajectory))
        self.yaw_error_trajectory = yaw_error_trajectory
        self.yaw_error_trajectory_label.setText(str(self.yaw_error_trajectory))

        self.dis_error_trajectory_goal = dis_error_trajectory_goal
        self.dis_error_trajectory_goal_label.setText(str(self.dis_error_trajectory_goal))
        self.yaw_error_trajectory_goal = yaw_error_trajectory_goal
        self.yaw_error_trajectory_goal_label.setText(str(self.yaw_error_trajectory_goal))

        self.dis_error_prekingpin = dis_error_prekingpin
        self.dis_error_prekingpin_label.setText(str(self.dis_error_prekingpin))
        self.yaw_error_prekingpin = yaw_error_prekingpin
        self.yaw_error_prekingpin_label.setText(str(self.yaw_error_prekingpin))

        self.dis_error_kingpin = dis_error_kingpin
        self.dis_error_kingpin_label.setText(str(self.dis_error_kingpin))
        self.yaw_error_kingpin = yaw_error_kingpin
        self.yaw_error_kingpin_label.setText(str(self.yaw_error_kingpin))


class TrajectoryGraph(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.graph_figure, self.graph_axis = plt.subplots(4,1,sharex=True)
        self.graph_figure_canvas = FigureCanvas(self.graph_figure)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.graph_figure_canvas)

        self.trajectory_vx, = self.graph_axis[0].plot([],[], '-g')
        self.graph_axis[0].set_ylabel('vx (m/s)')
        self.graph_axis[3].set_xlabel('length (m)')

        self.trajectory_ax, = self.graph_axis[1].plot([],[], '-g')
        self.graph_axis[1].set_ylabel('ax (m/s2)')
        self.graph_axis[3].set_xlabel('length (m)')

        self.trajectory_yaw, = self.graph_axis[2].plot([],[], '-g')
        self.graph_axis[2].set_ylabel('yaw (rad)')
        self.graph_axis[3].set_xlabel('length (m)')

        self.trajectory_curv, = self.graph_axis[3].plot([],[], '-g')
        self.graph_axis[3].set_ylabel('curvature (1/m)')
        self.graph_axis[3].set_xlabel('length (m)')
        
        for i in range(4):
            self.graph_axis[i].grid()

class Trajectory23Graph(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.graph23_figure, self.graph23_axis = plt.subplots(4,1,sharex=True)
        self.graph23_figure_canvas = FigureCanvas(self.graph23_figure)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.graph23_figure_canvas)

        self.trajectory_vx, = self.graph23_axis[0].plot([],[], '-r')
        self.graph23_axis[0].set_ylabel('vx (m/s)')
        self.graph23_axis[3].set_xlabel('length (m)')

        self.trajectory_ax, = self.graph23_axis[1].plot([],[], '-r')
        self.graph23_axis[1].set_ylabel('ax (m/s2)')
        self.graph23_axis[3].set_xlabel('length (m)')

        self.trajectory_yaw, = self.graph23_axis[2].plot([],[], '-r')
        self.graph23_axis[2].set_ylabel('yaw (rad)')
        self.graph23_axis[3].set_xlabel('length (m)')

        self.trajectory_curv, = self.graph23_axis[3].plot([],[], '-r')
        self.graph23_axis[3].set_ylabel('curvature (1/m)')
        self.graph23_axis[3].set_xlabel('length (m)')
        
        for i in range(4):
            self.graph23_axis[i].grid()

class CouplingScope(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.planner_data = PlannerData()

        # Create the tabs
        self.tabs = QtWidgets.QTabWidget(self)
        self.tab1 = StatusScreen()
        self.tab2 = TrajectoryGraph()
        self.tab3 = Trajectory23Graph()

        # Add the tabs to the tab widget
        self.tabs.addTab(self.tab1, "Status")
        self.tabs.addTab(self.tab2, "trajectory")
        self.tabs.addTab(self.tab3, "trajectory23")

        # Set the main widget to be the tab widget
        self.setCentralWidget(self.tabs)

    def update_status_screen(self):

        self.tab1.labelupdate(  round(self.planner_data.ego_x,2),round(self.planner_data.ego_y,2),round(np.rad2deg(self.planner_data.ego_yaw),2),
                                round(self.planner_data.ego_vx,2),round(self.planner_data.ego_curvature,2),
                                round(self.planner_data.prekingpin_x,2),round(self.planner_data.prekingpin_y,2),round(np.rad2deg(self.planner_data.prekingpin_yaw),2),
                                round(self.planner_data.kingpin_x,2),round(self.planner_data.kingpin_y,2),round(np.rad2deg(self.planner_data.kingpin_yaw),2),
                                round(self.planner_data.dis_error_trajectory,2),round(np.rad2deg(self.planner_data.yaw_error_trajectory),2),
                                round(self.planner_data.dis_error_trajectory_goal,2),round(np.rad2deg(self.planner_data.yaw_error_trajectory_goal),2),
                                round(self.planner_data.dis_error_prekingpin,2),round(np.rad2deg(self.planner_data.yaw_error_prekingpin),2),
                                round(self.planner_data.dis_error_kingpin,2),round(np.rad2deg(self.planner_data.yaw_error_kingpin),2)
                                )
    
    def full_update_data_bird_fig(self):
        #update data (with the new _and_ the old points)
        self.tab1.trajectory_axis.set_xdata([tpoint.x for tpoint in self.planner_data.trajectory])
        self.tab1.trajectory_axis.set_ydata([tpoint.y for tpoint in self.planner_data.trajectory])

        self.tab1.trajectory23_axis.set_xdata([tpoint.x for tpoint in self.planner_data.trajectory23])
        self.tab1.trajectory23_axis.set_ydata([tpoint.y for tpoint in self.planner_data.trajectory23])

        #update arrow data
        ego_length = 0.5
        pin_length = 0.35

        ego_Ax = self.planner_data.ego_x + (ego_length*np.cos(self.planner_data.ego_yaw))
        ego_Ay = self.planner_data.ego_y + (ego_length*np.sin(self.planner_data.ego_yaw))
        ego_Bx = ego_Ax + (ego_length*0.5*np.cos(self.planner_data.ego_yaw-np.deg2rad(150)))
        ego_By = ego_Ay + (ego_length*0.5*np.sin(self.planner_data.ego_yaw-np.deg2rad(150)))
        ego_Cx = ego_Ax + (ego_length*0.5*np.cos(self.planner_data.ego_yaw+np.deg2rad(150)))
        ego_Cy = ego_Ay + (ego_length*0.5*np.sin(self.planner_data.ego_yaw+np.deg2rad(150)))

        self.tab1.ego_arrow_line.set_xdata([self.planner_data.ego_x,ego_Ax])
        self.tab1.ego_arrow_line.set_ydata([self.planner_data.ego_y,ego_Ay])
        self.tab1.ego_arrow_poly.set_xy([   (ego_Ax,ego_Ay),
                                            (ego_Bx,ego_By),
                                            (ego_Cx,ego_Cy)])

        kingpin_Ax = self.planner_data.kingpin_x + (pin_length*np.cos(self.planner_data.kingpin_yaw))
        kingpin_Ay = self.planner_data.kingpin_y + (pin_length*np.sin(self.planner_data.kingpin_yaw))
        kingpin_Bx = kingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.kingpin_yaw-np.deg2rad(135)))
        kingpin_By = kingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.kingpin_yaw-np.deg2rad(135)))
        kingpin_Cx = kingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.kingpin_yaw+np.deg2rad(135)))
        kingpin_Cy = kingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.kingpin_yaw+np.deg2rad(135)))

        self.tab1.kingpin_line.set_xdata([self.planner_data.kingpin_x,kingpin_Ax])
        self.tab1.kingpin_line.set_ydata([self.planner_data.kingpin_y,kingpin_Ay])
        self.tab1.kingpin_arrow_poly.set_xy([   (kingpin_Ax,kingpin_Ay),
                                                (kingpin_Bx,kingpin_By),
                                                (kingpin_Cx,kingpin_Cy)])

        prekingpin_Ax = self.planner_data.prekingpin_x + (pin_length*np.cos(self.planner_data.prekingpin_yaw))
        prekingpin_Ay = self.planner_data.prekingpin_y + (pin_length*np.sin(self.planner_data.prekingpin_yaw))
        prekingpin_Bx = prekingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_By = prekingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_Cx = prekingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.prekingpin_yaw+np.deg2rad(135)))
        prekingpin_Cy = prekingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.prekingpin_yaw+np.deg2rad(135)))

        self.tab1.prekingpin_line.set_xdata([self.planner_data.prekingpin_x,prekingpin_Ax])
        self.tab1.prekingpin_line.set_ydata([self.planner_data.prekingpin_y,prekingpin_Ay])
        self.tab1.prekingpin_arrow_poly.set_xy([(prekingpin_Ax,prekingpin_Ay),
                                                (prekingpin_Bx,prekingpin_By),
                                                (prekingpin_Cx,prekingpin_Cy)])
 
    def lean_update_data_bird_fig(self):
        self.tab1.trajectory23_axis.set_xdata([tpoint.x for tpoint in self.planner_data.trajectory23])
        self.tab1.trajectory23_axis.set_ydata([tpoint.y for tpoint in self.planner_data.trajectory23])

        ego_length = 0.5
        pin_length = 0.35

        ego_Ax = self.planner_data.ego_x + (ego_length*np.cos(self.planner_data.ego_yaw))
        ego_Ay = self.planner_data.ego_y + (ego_length*np.sin(self.planner_data.ego_yaw))
        ego_Bx = ego_Ax + (ego_length*0.5*np.cos(self.planner_data.ego_yaw-np.deg2rad(150)))
        ego_By = ego_Ay + (ego_length*0.5*np.sin(self.planner_data.ego_yaw-np.deg2rad(150)))
        ego_Cx = ego_Ax + (ego_length*0.5*np.cos(self.planner_data.ego_yaw+np.deg2rad(150)))
        ego_Cy = ego_Ay + (ego_length*0.5*np.sin(self.planner_data.ego_yaw+np.deg2rad(150)))

        self.tab1.ego_arrow_line.set_xdata([self.planner_data.ego_x,ego_Ax])
        self.tab1.ego_arrow_line.set_ydata([self.planner_data.ego_y,ego_Ay])
        self.tab1.ego_arrow_poly.set_xy([   (ego_Ax,ego_Ay),
                                            (ego_Bx,ego_By),
                                            (ego_Cx,ego_Cy)])

        kingpin_Ax = self.planner_data.kingpin_x + (pin_length*np.cos(self.planner_data.kingpin_yaw))
        kingpin_Ay = self.planner_data.kingpin_y + (pin_length*np.sin(self.planner_data.kingpin_yaw))
        kingpin_Bx = kingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.kingpin_yaw-np.deg2rad(135)))
        kingpin_By = kingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.kingpin_yaw-np.deg2rad(135)))
        kingpin_Cx = kingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.kingpin_yaw+np.deg2rad(135)))
        kingpin_Cy = kingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.kingpin_yaw+np.deg2rad(135)))

        self.tab1.kingpin_line.set_xdata([self.planner_data.kingpin_x,kingpin_Ax])
        self.tab1.kingpin_line.set_ydata([self.planner_data.kingpin_y,kingpin_Ay])
        self.tab1.kingpin_arrow_poly.set_xy([   (kingpin_Ax,kingpin_Ay),
                                                (kingpin_Bx,kingpin_By),
                                                (kingpin_Cx,kingpin_Cy)])

        prekingpin_Ax = self.planner_data.prekingpin_x + (pin_length*np.cos(self.planner_data.prekingpin_yaw))
        prekingpin_Ay = self.planner_data.prekingpin_y + (pin_length*np.sin(self.planner_data.prekingpin_yaw))
        prekingpin_Bx = prekingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_By = prekingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.prekingpin_yaw-np.deg2rad(135)))
        prekingpin_Cx = prekingpin_Ax + (pin_length*0.5*np.cos(self.planner_data.prekingpin_yaw+np.deg2rad(135)))
        prekingpin_Cy = prekingpin_Ay + (pin_length*0.5*np.sin(self.planner_data.prekingpin_yaw+np.deg2rad(135)))

        self.tab1.prekingpin_line.set_xdata([self.planner_data.prekingpin_x,prekingpin_Ax])
        self.tab1.prekingpin_line.set_ydata([self.planner_data.prekingpin_y,prekingpin_Ay])
        self.tab1.prekingpin_arrow_poly.set_xy([(prekingpin_Ax,prekingpin_Ay),
                                                (prekingpin_Bx,prekingpin_By),
                                                (prekingpin_Cx,prekingpin_Cy)])

    def update_data_graph_fig(self):
        #Update data (with the new _and_ the old points)
        self.tab2.trajectory_vx.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory])
        self.tab2.trajectory_vx.set_ydata([tpoint.vx for tpoint in self.planner_data.trajectory])

        self.tab2.trajectory_ax.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory])
        self.tab2.trajectory_ax.set_ydata([tpoint.ax for tpoint in self.planner_data.trajectory])

        self.tab2.trajectory_yaw.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory])
        self.tab2.trajectory_yaw.set_ydata([tpoint.yaw for tpoint in self.planner_data.trajectory])

        self.tab2.trajectory_curv.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory])
        self.tab2.trajectory_curv.set_ydata([tpoint.curvature for tpoint in self.planner_data.trajectory])

    def update_data_graph23_fig(self):
        #Update data (with the new _and_ the old points)
        self.tab3.trajectory_vx.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory23])
        self.tab3.trajectory_vx.set_ydata([tpoint.vx for tpoint in self.planner_data.trajectory23])

        self.tab3.trajectory_ax.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory23])
        self.tab3.trajectory_ax.set_ydata([tpoint.ax for tpoint in self.planner_data.trajectory23])

        self.tab3.trajectory_yaw.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory23])
        self.tab3.trajectory_yaw.set_ydata([tpoint.yaw for tpoint in self.planner_data.trajectory23])

        self.tab3.trajectory_curv.set_xdata([tpoint.s for tpoint in self.planner_data.trajectory23])
        self.tab3.trajectory_curv.set_ydata([tpoint.curvature for tpoint in self.planner_data.trajectory23])
    
    def draw_bird_figure(self):

        self.planner_data.calc_limits()
        
        self.tab1.bird_axis.set_xlim(self.planner_data.xmin,self.planner_data.xmax)
        self.tab1.bird_axis.set_ylim(self.planner_data.ymin,self.planner_data.ymax)
        self.tab1.bird_axis.set_aspect('equal')
        #print(self.bird_axis.dataLim)
        
        self.tab1.bird_figure_canvas.draw()
        self.tab1.bird_figure_canvas.flush_events()

    def draw_graph_figure(self):

        for i in range(4):
            self.tab2.graph_axis[i].relim()
            self.tab2.graph_axis[i].autoscale_view()

        self.tab2.graph_figure.canvas.draw()
        self.tab2.graph_figure.canvas.flush_events()

    def draw_graph23_figure(self): 

        for i in range(4):
            self.tab3.graph23_axis[i].relim()
            self.tab3.graph23_axis[i].autoscale_view()

        self.tab3.graph23_figure.canvas.draw()
        self.tab3.graph23_figure.canvas.flush_events()