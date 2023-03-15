import numpy as np
import pyclothoids
from enum import Enum, auto
from scipy.integrate import quad
from scipy.optimize import fsolve
from scipy.misc import derivative

class TrajectoryPoint:
    def __init__(self,t=None,s=None,x=None,y=None,vx=None,ax=None,yaw=None,curvature=None):
        self.t = t
        self.s = s
        self.x = x   
        self.y = y
        self.vx = vx
        self.ax = ax
        self.yaw = yaw
        self.curvature = curvature

class Pose:
    def __init__(self,x=None, y=None, yaw=None, vx=None, curvature=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.curvature = curvature

class PlannerMode(Enum):
    STANDSTILL = auto()
    COUPLING_PHASE_PREKINGPIN = auto()
    COUPLING_PHASE_KINGPIN = auto()

class CouplingPlanner:

    ego_on_traj = 0
    drive_step = 0.2
    
    def __init__(self,  path_res=0.1, 
                        path23_res=0.1,
                        vx=-0.41, 
                        acc_time= 0.5,
                        dec_time= 0.5, 
                        history_point_limit=3, 
                        trajectory_backup=1,
                        ego_delta_bilevel=0.5, 
                        goal_delta_bilevel=0.5, 
                        max_curvature=0.26, 
                        min_traj_length=2, 
                        max_traj_length=15,
                        dis_prekingpin_kingpin=0.525):
        
        #trajectory parameter
        self.path_res = path_res
        self.path23_res = path23_res
        self.vx = vx
        self.acc_time = acc_time
        self.dec_time = dec_time
        self.history_point_limit = history_point_limit
        self.trajectory_backup = trajectory_backup
        self.ego_delta_bilevel = ego_delta_bilevel
        self.goal_delta_bilevel = goal_delta_bilevel
        self.max_curvature = max_curvature
        self.min_traj_length = min_traj_length
        self.max_traj_length = max_traj_length
        self.dis_prekingpin_kingpin = dis_prekingpin_kingpin

        #Simulation Pose
        self.kingpin_goal_pose = Pose(15.5, 20.2, np.deg2rad(90),0.0, 0.0)
        self.prekingpin_goal_pose = self.calc_prekingpin_pose(self.kingpin_goal_pose)

        self.goal_pose = Pose()
        self.ego_pose = Pose(5.8, 4.5, np.deg2rad(225),0.0, 0.0)

        self.planner_mode = None
        
        self.trajectory = []
        self.trajectory23 = []

    def update_pose(self, ego_pose, kingpin_pose):
        self.ego_pose = ego_pose

        self.kingpin_goal_pose = kingpin_pose
        self.prekingpin_goal_pose = self.calc_prekingpin_pose(kingpin_pose)

    def calc_prekingpin_pose(self,kingpin_pose):
        prekingpin_x = kingpin_pose.x + (self.dis_prekingpin_kingpin*np.cos(kingpin_pose.yaw))
        prekingpin_y = kingpin_pose.y + (self.dis_prekingpin_kingpin*np.sin(kingpin_pose.yaw))
        
        return Pose(prekingpin_x,prekingpin_y,kingpin_pose.yaw,kingpin_pose.vx,kingpin_pose.curvature)

    def taskmanager_cycle(self):
        
        if self.planner_mode is PlannerMode.STANDSTILL: 
            self.standstill()
        elif self.planner_mode is PlannerMode.COUPLING_PHASE_PREKINGPIN:
            self.prekingpin()
        elif self.planner_mode is PlannerMode.COUPLING_PHASE_KINGPIN:
            self.kingpin()
        else:
            self.standstill()

    def standstill(self):
        self.resample_trajectory23_standstill()

    def prekingpin(self):
        self.goal_pose = self.prekingpin_goal_pose

        if not self.trajectory:
            self.sample_g2straight_trajectory(self.ego_pose,self.goal_pose)

        self.bilevel_check()

        if self.feasibility_check():
            self.resample_trajectory23(self.trajectory)
        else:
            self.resample_trajectory23_standstill()

    def kingpin(self):
        self.goal_pose = self.kingpin_goal_pose

        if not self.trajectory:
            self.sample_straight_trajectory(self.prekingpin_goal_pose,self.goal_pose)
        self.resample_trajectory23(self.trajectory)

    def ego_drive_step(self,trajectory):

        closest_trajectory_point = self.give_closestprojection_on_trajectory(trajectory)
        self.ego_on_traj = closest_trajectory_point.s
        self.ego_on_traj += self.drive_step

        i = 1
        while i < len(trajectory):
            if trajectory[i-1].s <= self.ego_on_traj < trajectory[i].s:
                
                x = self.calc_lin_interpol(trajectory[i-1].s,trajectory[i].s,trajectory[i-1].x,trajectory[i].x,self.ego_on_traj)
                y = self.calc_lin_interpol(trajectory[i-1].s,trajectory[i].s,trajectory[i-1].y,trajectory[i].y,self.ego_on_traj)
                yaw = self.calc_lin_interpol_angle(trajectory[i-1].s,trajectory[i].s,trajectory[i-1].yaw,trajectory[i].yaw,self.ego_on_traj)

                self.ego_pose.x = x + np.random.normal(0,0.05)
                self.ego_pose.y = y + np.random.normal(0,0.05)
                self.ego_pose.yaw = yaw + np.random.normal(0,0.01)
                
                break
            i += 1

    def bilevel_check(self):

        dis_goal_trajgoal, theta_goal_trajgoal = self.calc_distance_angle_PoseA_PoseB(self.trajectory[-1],self.goal_pose)

        closest_trajectory_point = self.give_closestprojection_on_trajectory(self.trajectory)
        dis_ego_traj,theta_ego_traj = self.calc_distance_angle_PoseA_PoseB(closest_trajectory_point,self.ego_pose)

        if dis_goal_trajgoal > self.goal_delta_bilevel or dis_ego_traj > self.ego_delta_bilevel:
            self.sample_g2straight_trajectory(self.ego_pose,self.goal_pose)

    def feasibility_check(self):

        curvature_feasible = True
        for trajectory_point in self.trajectory:
            if abs(trajectory_point.curvature) > self.max_curvature:
                curvature_feasible = False

        if self.min_traj_length <= self.trajectory[-1].s <= self.max_traj_length and curvature_feasible:
            return True
        else:
            return False

    def sample_g2straight_trajectory(self,start_pose, goal_pose):

        self.trajectory.clear()
        trajectory = []

        if self.vx < 0.0:

            trajectory = self.sample_g2straight_path(   Pose(   x=start_pose.x,
                                                                y=start_pose.y,
                                                                yaw=start_pose.yaw +np.pi,
                                                                vx=start_pose.vx,
                                                                curvature=start_pose.curvature),
                                                        Pose(   x=goal_pose.x,
                                                                y=goal_pose.y,
                                                                yaw =goal_pose.yaw+np.pi,
                                                                vx=goal_pose.vx,
                                                                curvature=goal_pose.curvature),
                                                                self.trajectory_backup)
            self.offset_yaw_backward(trajectory)
            self.offset_curvature_backward(trajectory)
        else:
            trajectory = self.sample_g2straight_path(start_pose,goal_pose,self.trajectory_backup)
            self.offset_yaw_forward(trajectory)
        
        self.calc_trajectory(trajectory)
        self.trajectory = trajectory

    def sample_straight_trajectory(self,start_pose, goal_pose):

        self.trajectory.clear()
        trajectory = []

        if self.vx < 0.0:
            trajectory = self.sample_straight(start_pose.x,start_pose.y,goal_pose.x,goal_pose.y)
            self.offset_yaw_backward(trajectory)
            self.offset_curvature_backward(trajectory)
        else:
            trajectory = self.sample_straight(start_pose.x,start_pose.y,goal_pose.x,goal_pose.y)
            self.offset_yaw_forward(trajectory)
        
        self.calc_trajectory(trajectory)
        self.trajectory = trajectory

    def sample_g2straight_path(self,start_pose,end_pose,straight_length):

        mid_x = end_pose.x - (straight_length*np.cos(self.angle_interval(end_pose.yaw)))
        mid_y = end_pose.y - (straight_length*np.sin(self.angle_interval(end_pose.yaw)))

        mid_pose = Pose(x=mid_x,y=mid_y,yaw=end_pose.yaw,vx=None,curvature=end_pose.curvature)

        path_0 = self.sample_g2path(start_pose,mid_pose)
        path_1 = self.sample_straight(mid_pose.x,mid_pose.y,end_pose.x,end_pose.y)

        for point in path_1:
            point.s += path_0[-1].s

        if path_1:
            path_1.pop(0)

        return path_0 + path_1

    def sample_g2path(self,start_pose,end_pose):

        g2clothoid_list = pyclothoids.SolveG2(  start_pose.x, start_pose.y, start_pose.yaw, start_pose.curvature,\
                                                end_pose.x, end_pose.y, end_pose.yaw, end_pose.curvature)

        path_0 = self.sample_clothoid(g2clothoid_list[0])
        path_1 = self.sample_clothoid(g2clothoid_list[1])
        path_2 = self.sample_clothoid(g2clothoid_list[2])

        for point in path_1:
            point.s += g2clothoid_list[0].length

        for point in path_2:
            point.s += (g2clothoid_list[0].length + g2clothoid_list[1].length)

        if path_1:
            path_1.pop(0)

        if path_2:
            path_2.pop(0)

        return path_0 + path_1 + path_2

    def sample_clothoid(self,clothoid):

        n_sample_points = round(clothoid.length/self.path_res)
        sample_points = np.round(np.linspace(0.0,clothoid.length,n_sample_points),4)

        path = []

        for sample_point in sample_points:
            path.append(TrajectoryPoint(  
                s = round(sample_point,4),
                x = round(clothoid.X(sample_point),4),
                y = round(clothoid.Y(sample_point),4),
                yaw = round(clothoid.Theta(sample_point),4),
                curvature = round((clothoid.KappaStart + (clothoid.dk*sample_point)),4)))

        return path

    def sample_straight(self,x_start,y_start,x_end,y_end):

        dx = x_end - x_start
        dy = y_end - y_start
        length = np.hypot(dy, dx)
        theta = np.arctan2(dy, dx)

        n_sample_points = round(length/self.path_res)
        sample_points = np.round(np.linspace(0.0,length,n_sample_points),4)

        path = []

        for sample_point in sample_points:
            path.insert(0,TrajectoryPoint(  
                s = round(length-sample_point,4),
                x = round(x_end - (sample_point*np.cos(theta)),4),
                y = round(y_end - (sample_point*np.sin(theta)),4),
                yaw = round(theta,4),
                curvature = round(0.0,4)))

        return path


    def resample_trajectory23(self, traj):

        self.trajectory23.clear()

        #calculate length startpoint on main trajectory
        closest_trajectory_point = self.give_closestprojection_on_trajectory(traj)
        zero_len_on_traj = closest_trajectory_point.s 

        #calculate time startpint on main trajectory
        j = 1
        while j < len(traj):
            if traj[j-1].s <= zero_len_on_traj <= traj[j].s:
                zero_time_on_traj = self.calc_lin_interpol( traj[j-1].s,
                                                            traj[j].s,
                                                            traj[j-1].t,
                                                            traj[j].t,
                                                            zero_len_on_traj)
            j += 1

        #store trajectory data in lists
        t_raw = []
        s_raw = []
        x_raw = []
        y_raw = []
        yaw_raw = []
        vx_raw = []
        ax_raw = []
        curv_raw = []

        for traj_point in traj:
            t_raw.append(traj_point.t)
            s_raw.append(traj_point.s)
            x_raw.append(traj_point.x)
            y_raw.append(traj_point.y)
            yaw_raw.append(traj_point.yaw)
            vx_raw.append(traj_point.vx)
            ax_raw.append(traj_point.ax)
            curv_raw.append(traj_point.curvature)

        #resample trajectory23
        t_23 = []
        s_23 = []
        x_23 = []
        y_23 = []
        yaw_23 = []
        vx_23 = []
        ax_23 = []
        curv_23 = []

        for i in np.arange( -self.path23_res*self.history_point_limit,
                            (23-self.history_point_limit)*self.path23_res,
                            self.path23_res):

            s_23.append(i+zero_len_on_traj)

        t_23 = np.interp(s_23,s_raw,t_raw,-np.inf,np.inf)
        x_23 = np.interp(s_23,s_raw,x_raw)
        y_23 = np.interp(s_23,s_raw,y_raw)
        #yaw_23 = np.interp(s_23,s_raw,yaw_raw)
        yaw_23 = self.angle_interp(s_23,s_raw,yaw_raw)
        vx_23 = np.interp(s_23,s_raw,vx_raw,0.,0.)
        ax_23 = np.interp(s_23,s_raw,ax_raw,0.,0.)
        curv_23 = np.interp(s_23,s_raw,curv_raw)
        s_23 = np.interp(s_23,s_raw,s_raw)

        for i in range(23):
            s_23[i] = round(s_23[i]-zero_len_on_traj,4)

        for i in range(23):
            t_23[i] = round(t_23[i]-zero_time_on_traj,4)        
        
        for i in np.arange(0,23,1):
            if t_23[i] == np.inf:
                t_23[i] = t_23[i-1] + abs(self.path23_res/self.vx)
        
        for i in np.arange(22,-1,-1):
            if t_23[i] == -np.inf:
                t_23[i] = t_23[i+1] - abs(self.path23_res/self.vx)

        for i in range(23):
            self.trajectory23.append(TrajectoryPoint(t=t_23[i],s=s_23[i],x=x_23[i],y=y_23[i],vx=vx_23[i],ax=ax_23[i],yaw=yaw_23[i],curvature=curv_23[i]))
    
    def resample_trajectory23_standstill(self):
        trajectory_point = TrajectoryPoint(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0)
        stillstand_trajectory = []
        for i in range(23):
            stillstand_trajectory.append(trajectory_point)
        
        self.trajectory23 = stillstand_trajectory

    def offset_yaw_forward(self,trajectory):

        for trajectory_point in trajectory:
            trajectory_point.yaw = self.angle_interval(trajectory_point.yaw)

    def offset_yaw_backward(self,trajectory):

        for trajectory_point in trajectory:
            trajectory_point.yaw = self.angle_interval(trajectory_point.yaw-np.pi)

    def offset_curvature_backward(self,trajectory):

        for trajectory_point in trajectory:
            trajectory_point.curvature = -trajectory_point.curvature

    def func_acc(self,t):
        return (((self.vx-self.ego_pose.vx)/self.acc_time)*t)+self.ego_pose.vx
    
    def func_dec(self,t):
        return ((-self.vx/self.dec_time)*(t))+self.vx

    def func_acc_abs(self,t):
        return (((abs(self.vx)-abs(self.ego_pose.vx))/self.acc_time)*t)+abs(self.ego_pose.vx)
    
    def func_dec_abs(self,t):
        return ((-abs(self.vx)/self.dec_time)*(t))+abs(self.vx)
        
    def calc_trajectory(self,trajectory):    
        
        len_on_traj = 0
        
        def integral_acc(t):
            integral,err = quad(self.func_acc_abs,0,t)
            return len_on_traj-integral
        
        def integral_dec(t):
            integral,err = quad(self.func_dec_abs,0,t)
            return len_on_traj-integral

        ds1,_ = quad(self.func_acc_abs,0,self.acc_time)
        ds3,_ = quad(self.func_dec_abs,0,self.dec_time)
        ds2 = trajectory[-1].s-ds1-ds3

        dt1 = self.acc_time
        dt2 = ds2/abs(self.vx)
        dt3 = self.dec_time

        vfunc_acc = np.vectorize(integral_acc)
        vfunc_dec = np.vectorize(integral_dec)

        #solve timestamps for path resolution 

        i=0
        while i < len(trajectory):

            if trajectory[i].s < ds1:
                len_on_traj = trajectory[i].s
                res, = fsolve(vfunc_acc,0.01)
                trajectory[i].t = round(res,4)
                i += 1

            if ds1 <= trajectory[i].s <= ds1 + ds2:
                len_on_traj = trajectory[i].s -ds1
                res = len_on_traj/abs(self.vx)
                trajectory[i].t = round(dt1+res,4)
                i += 1

            if ds1 + ds2 < trajectory[i].s:
                len_on_traj = trajectory[i].s -ds1-ds2
                res, = fsolve(vfunc_dec,0.01)
                trajectory[i].t = round(dt1+dt2+res,4)
                i += 1
        
        #calculate velocity values based on timestamps
        i=0
        while i < len(trajectory):
            
            if trajectory[i].t <= dt1:
                trajectory[i].vx = round(self.func_acc(trajectory[i].t),4)
                i += 1

            if dt1 < trajectory[i].t <= dt1 + dt2:    
                trajectory[i].vx = round(self.vx,4)
                i += 1

            if dt1 + dt2 <= trajectory[i].t:
                trajectory[i].vx = round(self.func_dec(trajectory[i].t-dt1-dt2),4)
                i += 1

        #if self.vx < 0.0:
        #    trajectory[0].vx -= 0.001
        #else: 
        #    trajectory[0].vx += 0.001 

        #calculate acceleration values based on velocity derivation
        i=1
        while i < len(trajectory):
            
            if trajectory[i].t <= dt1:
                trajectory[i].ax = round(derivative(self.func_acc,trajectory[i-1].t,trajectory[i].t-trajectory[i-1].t),4)
                i += 1

            if dt1 < trajectory[i].t <= dt1 + dt2:    
                trajectory[i].ax = round(0.0,4)
                i += 1

            if dt1 + dt2 <= trajectory[i].t:
                trajectory[i].ax = round(derivative(self.func_dec,trajectory[i-1].t,trajectory[i].t-trajectory[i-1].t),4)
                i += 1
        trajectory[0].ax = trajectory[1].ax
        #trajectory[-1].ax = 0.0

    def give_closestprojection_on_trajectory(self,trajectory):

        closest_trajpoint = trajectory[0]

        for trajpoint in trajectory:

            if self.calc_distance_angle_PoseA_PoseB(trajpoint,self.ego_pose)[0] < self.calc_distance_angle_PoseA_PoseB(closest_trajpoint,self.ego_pose)[0]:
                closest_trajpoint = trajpoint

        return closest_trajpoint

    def give_latprojection_on_trajectory(self,trajectory):

        projection_yaw = np.deg2rad((np.rad2deg(self.ego_pose.yaw)+360+90)%360)

        projection_x1 = self.ego_pose.x
        projection_y1 = self.ego_pose.y 
        projection_x2 = self.ego_pose.x + (np.cos(projection_yaw))
        projection_y2 = self.ego_pose.y + (np.sin(projection_yaw))

        i = 1

        while i < len(trajectory):
            
            x,y = self.find_intersection(trajectory[i-1].x,trajectory[i].x,trajectory[i-1].y,trajectory[i].y,\
                                        projection_x1,projection_x2,projection_y1,projection_y2)

            x = round(x,4)
            y = round(y,4)

            if ((trajectory[i-1].x <= x <= trajectory[i].x or trajectory[i-1].x >= x >= trajectory[i].x) and 
                (trajectory[i-1].y >= y >= trajectory[i].y or trajectory[i-1].y <= y <= trajectory[i].y)):

                return trajectory[i-1]

            i += 1

        return self.give_closestprojection_on_trajectory(trajectory)

    def calc_distance_angle_PoseA_PoseB(self,PoseA,PoseB):
        dx = PoseA.x - PoseB.x
        dy = PoseA.y - PoseB.y
        d = np.hypot(dx, dy)
        theta = self.angle_interval(PoseB.yaw-PoseA.yaw)
        return d, theta

    def angle_interp(self,x23, xp, yp):

        y23 = []

        for x3 in x23:

            x1 = 0
            x2 = 0
            x1_elem = None
            x2_elem = None

            i=0
            while i<len(xp):
                if xp[i] <= x3:
                    x1 = xp[i]
                    x1_elem = i
                i+=1
                
            j=len(xp)-1
            while j > 0:
                if xp[j] > x3:
                    x2 = xp[j]
                    x2_elem = j
                j-=1

            if x3 < x1:
                y23.append(yp[0])
            elif x3 > x2:
                y23.append(yp[-1])
            else:
                y23.append(self.calc_lin_interpol_angle(x1,x2,yp[x1_elem],yp[x2_elem],x3))

        return y23

    def calc_lin_interpol_angle(self,x1,x2,y1,y2,x3):

        y1 = (np.rad2deg(y1)+360)%360
        y2 = (np.rad2deg(y2)+360)%360

        max_v = max(y1,y2)
        min_v = min(y1,y2)

        propA = max_v-min_v
        propB = 360-propA
        propF = min(propA,propB)

        xp = [x1,x2]
        fp = [0.0,propF]
        delta = np.interp(x3,xp,fp)

        if y1 <= y2:
            if propF == propA:
                interpolated_v = min_v+delta
            elif propF == propB:
                interpolated_v = max_v+delta
        else:
            if propF == propA:
                interpolated_v = max_v-delta
            elif propF == propB:
                interpolated_v = min_v-delta
        
        interpolated_v = np.deg2rad(interpolated_v)

        return self.angle_interval(interpolated_v)
            
    @staticmethod
    def calc_lin_interpol(x1,x2,y1,y2,x3):
        m = (y2-y1)/(x2-x1)   
        return (m*x3)+(y1-(m*x1))

    @staticmethod
    def find_intersection(LineA_x1,LineA_x2,LineA_y1,LineA_y2,LineB_x1,LineB_x2,LineB_y1,LineB_y2):

        xdiff = (LineA_x1 - LineA_x2, LineB_x1 - LineB_x2)
        ydiff = (LineA_y1 - LineA_y2, LineB_y1 - LineB_y2)

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            raise Exception('lines do not intersect')
            #return None

        d = (det((LineA_x1,LineA_y1),(LineA_x2,LineA_y2)),\
            det((LineB_x1, LineB_y1),(LineB_x2,LineB_y2)))

        x = det(d, xdiff) / div
        y = det(d, ydiff) / div

        return x,y

    @staticmethod
    def angle_interval(angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

    @staticmethod
    def divide(a,b):
        if b == 0.0:
            return 0.0
        else:
            return a/b