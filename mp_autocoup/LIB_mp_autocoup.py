import numpy as np
import pyclothoids
from enum import Enum, auto
from scipy.integrate import quad
from scipy.optimize import fsolve
from scipy.misc import derivative
from VIS_mp_autocoup import AutocoupAnimation

show_animation = 2
simulate_ego = True

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
    def __init__(self, t=None,x=None, y=None, yaw=None, vx=None, curvature=None):
        self.t = t
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.curvature = curvature

class PlannerMode(Enum):
    STANDSTILL = auto()
    COUPLING_PHASE_TILL_PREKINGPIN = auto()
    COUPLING_PHASE_TILL_KINGPIN = auto()

class AutocoupPlanner:

    ego_pose = Pose()
    prekingpin_pose = Pose()
    kingpin_pose = Pose()

    ego_on_traj = 0
    drive_step = 0.2
    
    def __init__(self,  path_res=0.1, path23_res=0.1, vx=-0.41, acc_dec_time=0.5, history_point_limit=3, trajectory_backup=1,
                        ego_delta_bilevel=0.5, goal_delta_bilevel=0.15, max_curvature=0.26, min_traj_length=2,
                        dis_prekingpin_kingpin=2):
        
        #trajectory parameter
        self.path_res = path_res
        self.path23_res = path23_res
        self.vx = vx
        self.acc_dec_time = acc_dec_time
        self.history_point_limit = history_point_limit
        self.trajectory_backup = trajectory_backup
        
        #planner parameter
        self.ego_delta_bilevel = ego_delta_bilevel
        self.goal_delta_bilevel = goal_delta_bilevel
        self.max_curvature = max_curvature
        self.min_traj_length = min_traj_length

        #physical parameter
        self.dis_prekingpin_kingpin = dis_prekingpin_kingpin
        
        self.planner_mode = PlannerMode.COUPLING_PHASE_TILL_PREKINGPIN

        self.trajectory_p1 = []
        self.trajectory_p2 = []
        self.trajectory23 = []
        self.trajectory_invalid = []

        self.animation = AutocoupAnimation()

    def update_pose(self, ego_pose, kingpin_pose):
        self.ego_pose = ego_pose
        self.kingpin_pose = kingpin_pose
        self.prekingpin_pose = self.calc_prekingpin_pose(self.kingpin_pose)

    def calc_prekingpin_pose(self,kingpin_pose):
        prekingpin_x = kingpin_pose.x + (self.dis_prekingpin_kingpin*np.cos(kingpin_pose.yaw))
        prekingpin_y = kingpin_pose.y + (self.dis_prekingpin_kingpin*np.sin(kingpin_pose.yaw))
        
        return Pose(kingpin_pose.t,prekingpin_x,prekingpin_y,kingpin_pose.yaw,kingpin_pose.vx,kingpin_pose.curvature)

    def cycle(self):
        
        if self.pose_validation():
            print("pose_validation",end=' -> ')

            if self.planner_mode is PlannerMode.STANDSTILL:
                print("standstill" , end=' -> ')

                if show_animation:
                    self.visualization()
                
                print("accomplished")

            elif self.planner_mode is PlannerMode.COUPLING_PHASE_TILL_PREKINGPIN:
                print("phase_1", end=' -> ')
                
                if not self.trajectory_p1 or not self.trajectory_p2:
                    self.sample_trajectory_p1()
                    self.sample_trajectory_p2()

                self.bilevel_check()
                if self.feasibility_check():
                    self.resample_trajectory23(self.trajectory_p1)
                    
                    if simulate_ego:
                        self.ego_drive_step(self.trajectory23)
                    if show_animation:
                        self.visualization()

                    print("accomplished")
                else:
                    #send invalid trajectory
                    print("abort_mission") 
                    
            
            elif self.planner_mode is PlannerMode.COUPLING_PHASE_TILL_KINGPIN:
                print("phase_2")

                if not self.trajectory_p2:
                    self.sample_trajectory_p2()

                self.resample_trajectory23(self.trajectory_p2)

                if show_animation:
                    self.visualization()
                if simulate_ego:
                    self.ego_drive_step(self.trajectory23)
                
                print("accomplished")
            else:
                #send invalid trajectory
                print("invalid_phase")
        else:
            #send invalid trajectory
            print("invalid_pose")

    def visualization(self):

        self.animation.data_transfer(self.trajectory_p1,self.trajectory_p2,self.trajectory23)
        
        if show_animation == 2:
            self.animation.update_data_pose(    self.ego_pose.x,self.ego_pose.y,self.ego_pose.yaw,\
                                                self.kingpin_pose.x,self.kingpin_pose.y,self.kingpin_pose.yaw,\
                                                self.prekingpin_pose.x,self.prekingpin_pose.y,self.prekingpin_pose.yaw
                                                )
        
    def ego_drive_step(self,traj):

        _, self.ego_on_traj = self.give_closestprojection_on_trajectory(traj)
        self.ego_on_traj += self.drive_step

        i = 1
        while i < len(traj):
            if traj[i-1].s <= self.ego_on_traj < traj[i].s:
                
                x = self.calc_lin_interpol(traj[i-1].s,traj[i].s,traj[i-1].x,traj[i].x,self.ego_on_traj)
                y = self.calc_lin_interpol(traj[i-1].s,traj[i].s,traj[i-1].y,traj[i].y,self.ego_on_traj)
                yaw =self.calc_lin_interpol_angle(traj[i-1].s,traj[i].s,traj[i-1].yaw,traj[i].yaw,self.ego_on_traj)

                self.ego_pose.x = x + np.random.normal(0,0.1)
                self.ego_pose.y = y + np.random.normal(0,0.1)
                self.ego_pose.yaw = yaw #+ np.random.normal(0,0.01)
                
                break       
            i += 1

    def pose_validation(self):
        return True

    def bilevel_check(self):

        dis_goal_trajgoal, theta_goal_trajgoal = self.calc_distance_angle_PoseA_PoseB(self.trajectory_p1[-1],self.prekingpin_pose)
        point_on_trajectory,_ = self.give_closestprojection_on_trajectory(self.trajectory_p1)
        dis_ego_traj,theta_ego_traj = self.calc_distance_angle_PoseA_PoseB(point_on_trajectory,self.ego_pose)

        if dis_goal_trajgoal > self.goal_delta_bilevel and dis_ego_traj > self.ego_delta_bilevel:
            #print("failed: ego or goal not on trajectory")
            self.sample_trajectory_p1()
            self.sample_trajectory_p2()

    def feasibility_check(self):

        curvature_feasible = True
        for trajectory_point in self.trajectory_p1:
            if abs(trajectory_point.curvature) > self.max_curvature:
                curvature_feasible = False

        if self.trajectory_p1[-1].s > self.min_traj_length and curvature_feasible:
            return True
        else:
            return False

    def sample_trajectory_p1(self):

        self.trajectory_p1.clear()

        #rotate angles by 180 (backward driving)
        ego_calc_angle = self.angle_interval(self.ego_pose.yaw+np.pi)
        preprekingpin_calc_angel = self.angle_interval(self.prekingpin_pose.yaw+np.pi)
        
        #add 1m linear path (controller backup)
        preprekingpin_calc_x = self.prekingpin_pose.x - (self.trajectory_backup*np.cos(preprekingpin_calc_angel))
        preprekingpin_calc_y = self.prekingpin_pose.y - (self.trajectory_backup*np.sin(preprekingpin_calc_angel))

        g2clothoid_list = pyclothoids.SolveG2(self.ego_pose.x, self.ego_pose.y, ego_calc_angle, self.ego_pose.curvature,\
                                                preprekingpin_calc_x, preprekingpin_calc_y, preprekingpin_calc_angel, 0)

        clothoid_length = g2clothoid_list[0].length + g2clothoid_list[1].length + g2clothoid_list[2].length
        total_length =  clothoid_length + self.trajectory_backup
        npt_tar = round(total_length/self.path_res)
        samp_int = total_length/npt_tar

        samp_point = 0
        
        while samp_point <= g2clothoid_list[0].length and len(self.trajectory_p1) <= npt_tar:

            self.trajectory_p1.append(TrajectoryPoint(  s=round(samp_point,4),
                                                        x=round(g2clothoid_list[0].X(samp_point),4),
                                                        y=round(g2clothoid_list[0].Y(samp_point),4),
                                                        yaw=round(g2clothoid_list[0].Theta(samp_point),4),
                                                        curvature=round((g2clothoid_list[0].KappaStart + (g2clothoid_list[0].dk*samp_point)),4)))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length <= g2clothoid_list[1].length and len(self.trajectory_p1) <= npt_tar:

            self.trajectory_p1.append(TrajectoryPoint(  s=round(samp_point,4),
                                                        x=round(g2clothoid_list[1].X(samp_point - g2clothoid_list[0].length),4),
                                                        y=round(g2clothoid_list[1].Y(samp_point - g2clothoid_list[0].length),4),
                                                        yaw=round(g2clothoid_list[1].Theta(samp_point - g2clothoid_list[0].length),4),
                                                        curvature=round((g2clothoid_list[1].KappaStart + (g2clothoid_list[1].dk*(samp_point-g2clothoid_list[0].length))),4)))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length <= g2clothoid_list[2].length and len(self.trajectory_p1) <= npt_tar:
            
            self.trajectory_p1.append(TrajectoryPoint(  s=round(samp_point,4),
                                                        x=round(g2clothoid_list[2].X(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                        y=round(g2clothoid_list[2].Y(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                        yaw=round(g2clothoid_list[2].Theta(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                        curvature=round((g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length))),4)))

            samp_point += samp_int

        while samp_point < total_length and len(self.trajectory_p1) <= npt_tar:

            self.trajectory_p1.append(TrajectoryPoint(  s=round(samp_point,4),
                                                        x = round(preprekingpin_calc_x + ((samp_point-clothoid_length) * np.cos(preprekingpin_calc_angel)),4),
                                                        y = round(preprekingpin_calc_y + ((samp_point-clothoid_length) * np.sin(preprekingpin_calc_angel)),4),
                                                        yaw = round(preprekingpin_calc_angel,4),
                                                        curvature=0))

            samp_point += samp_int

        self.offset_yaw(self.trajectory_p1)
        self.add_long2trajectory(self.trajectory_p1)

    def sample_trajectory_p2(self):

        self.trajectory_p2.clear()

        dis_prekingpin_kingpin,_ = self.calc_distance_angle_PoseA_PoseB(self.prekingpin_pose,self.kingpin_pose)
        total_length = dis_prekingpin_kingpin + 2
        npt_tar = round(total_length/self.path_res)
        samp_int = total_length/npt_tar

        traj_yaw = self.angle_interval(self.prekingpin_pose.yaw+np.pi)

        samp_point = 0

        while samp_point < total_length and len(self.trajectory_p2) <= npt_tar:

            self.trajectory_p2.append(TrajectoryPoint(  s=round(samp_point,4),
                                                        x = round(self.prekingpin_pose.x + (samp_point * np.cos(traj_yaw)),4),
                                                        y = round(self.prekingpin_pose.y + (samp_point * np.sin(traj_yaw)),4),
                                                        yaw = round(traj_yaw,4),
                                                        curvature=0))

            samp_point += samp_int

        self.offset_yaw(self.trajectory_p2)
        self.add_long2trajectory(self.trajectory_p2)

    def resample_trajectory23(self,traj):

        self.trajectory23.clear()

        #calculate length startpoint on main trajectory
        _,zero_len_on_traj = self.give_closestprojection_on_trajectory(traj)

        #calculate time startpint on main trajectory
        j = 1
        while j < len(traj):

            if traj[j-1].s <= zero_len_on_traj < traj[j].s:
                zero_time_on_traj = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].t,traj[j].t,zero_len_on_traj)

            j += 1

        #resample prediction trajectory
        j = 1
        prediction_cnt = 0
        len_on_traj = zero_len_on_traj + (prediction_cnt * self.path23_res)
        
        while j < len(traj) and prediction_cnt < (23 - self.history_point_limit):

            if traj[j-1].s <= len_on_traj < traj[j].s:

                self.trajectory23.append(TrajectoryPoint(   t = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].t,traj[j].t,len_on_traj) - zero_time_on_traj,
                                                            s = prediction_cnt * self.path23_res,
                                                            x = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].x,traj[j].x,len_on_traj),
                                                            y = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].y,traj[j].y,len_on_traj),
                                                            vx = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].vx,traj[j].vx,len_on_traj),
                                                            ax = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].ax,traj[j].ax,len_on_traj),
                                                            yaw = self.calc_lin_interpol_angle(traj[j-1].s,traj[j].s,traj[j-1].yaw,traj[j].yaw,len_on_traj),
                                                            curvature= self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].curvature,traj[j].curvature,len_on_traj)
                                                            ))

                j = 0
                prediction_cnt += 1
                len_on_traj = zero_len_on_traj + (prediction_cnt * self.path23_res)
            
            elif zero_len_on_traj + (prediction_cnt * self.path23_res) > traj[-1].s:

                self.trajectory23.append(TrajectoryPoint(   t = self.trajectory23[-1].t + abs(self.path23_res/self.vx),
                                                            s = self.trajectory23[-1].s,
                                                            x = self.trajectory23[-1].x,
                                                            y = self.trajectory23[-1].y,
                                                            vx = 0.,
                                                            ax = 0.,
                                                            yaw = self.trajectory23[-1].yaw,
                                                            curvature= self.trajectory23[-1].curvature                                               
                                                            ))
                
                j = 0
                prediction_cnt += 1
                len_on_traj = zero_len_on_traj + (prediction_cnt * self.path23_res)
            
            j += 1
        

        #resample historic trajectory
        j = 1
        history_cnt = 1
        len_on_traj = zero_len_on_traj - (history_cnt * self.path23_res)

        while j < len(traj) and history_cnt <= self.history_point_limit:

            if traj[j-1].s <= len_on_traj < traj[j].s:

                self.trajectory23.insert(0,TrajectoryPoint( t = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].t,traj[j].t,len_on_traj) - zero_time_on_traj,
                                                            s = -history_cnt * self.path23_res,
                                                            x = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].x,traj[j].x,len_on_traj),
                                                            y = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].y,traj[j].y,len_on_traj),
                                                            vx = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].vx,traj[j].vx,len_on_traj),
                                                            ax = self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].ax,traj[j].ax,len_on_traj),
                                                            yaw = self.calc_lin_interpol_angle(traj[j-1].s,traj[j].s,traj[j-1].yaw,traj[j].yaw,len_on_traj),
                                                            curvature= self.calc_lin_interpol(traj[j-1].s,traj[j].s,traj[j-1].curvature,traj[j].curvature,len_on_traj)
                                                            ))

                j = 0
                history_cnt += 1
                len_on_traj = zero_len_on_traj - (history_cnt * self.path23_res)

            elif zero_len_on_traj - (history_cnt * self.path23_res) < traj[0].s:

                self.trajectory23.insert(0,TrajectoryPoint( t = self.trajectory23[0].t - abs(self.path23_res/self.vx),
                                                            s = self.trajectory23[0].s,
                                                            x = self.trajectory23[0].x,
                                                            y = self.trajectory23[0].y,
                                                            vx = 0.,
                                                            ax = 0.,
                                                            yaw = self.trajectory23[0].yaw,
                                                            curvature= self.trajectory23[0].curvature                                                
                                                            ))
                
                j = 0
                history_cnt += 1
                len_on_traj = zero_len_on_traj + (history_cnt * self.path23_res)
            
            j += 1

    def resample_trajectory23_old(self,trajectory):

        self.trajectory23.clear()

        #_,length_on_trajectory = self.give_latprojection_on_trajectory()
        _,length_on_trajectory = self.give_closestprojection_on_trajectory(trajectory)

        j = 1
        trajectory23_cnt = 0

        length_offset = length_on_trajectory
        time_offset = self.calc_lin_interpol(trajectory[j-1].s,trajectory[j].s,\
                                                trajectory[j-1].t,trajectory[j].t,length_on_trajectory)

        while j < len(trajectory) and trajectory23_cnt < 23:
            
            if trajectory[j-1].s <= length_on_trajectory < trajectory[j].s:

                new_traj_point = TrajectoryPoint(s=(self.path23_res*trajectory23_cnt))

                new_traj_point.t = (self.calc_lin_interpol(trajectory[j-1].s,trajectory[j].s,\
                                                            trajectory[j-1].t,trajectory[j].t,length_on_trajectory) - time_offset)

                new_traj_point.x = self.calc_lin_interpol(trajectory[j-1].s,trajectory[j].s,\
                                                            trajectory[j-1].x,trajectory[j].x,length_on_trajectory)

                new_traj_point.y = self.calc_lin_interpol(trajectory[j-1].s,trajectory[j].s,\
                                                            trajectory[j-1].y,trajectory[j].y,length_on_trajectory)
                
                new_traj_point.yaw = self.calc_lin_interpol_angle(trajectory[j-1].s,trajectory[j].s,\
                                                                    trajectory[j-1].yaw,trajectory[j].yaw,length_on_trajectory)

                new_traj_point.curvature = self.calc_lin_interpol(trajectory[j-1].s,trajectory[j].s,\
                                                                    trajectory[j-1].curvature,trajectory[j].curvature,length_on_trajectory)

                new_traj_point.vx = self.calc_lin_interpol(trajectory[j-1].s,trajectory[j].s,\
                                                            trajectory[j-1].vx,trajectory[j].vx,length_on_trajectory)
                
                new_traj_point.ax = self.calc_lin_interpol(trajectory[j-1].s,trajectory[j].s,\
                                                            trajectory[j-1].ax,trajectory[j].ax,length_on_trajectory)

                self.trajectory23.append(new_traj_point)

                length_on_trajectory += self.path23_res
                trajectory23_cnt += 1
                j = 0
                
            j += 1

    def offset_yaw(self,trajectory):

        for trajectory_point in trajectory:
            trajectory_point.yaw = self.angle_interval(trajectory_point.yaw-np.pi)
    
    def add_long2trajectory(self,trajectory):

        #calculate time & path boundaries

        vx_pos = abs(self.vx)
        ego_vx_pos = abs(self.ego_pose.vx)

        cc_time = self.acc_dec_time

        acc_profile = lambda x: (((vx_pos-ego_vx_pos)/cc_time)*x)+ego_vx_pos
        const_profile = lambda x: (vx_pos*x)
        dec_profile = lambda x: ((-vx_pos/cc_time)*(x))+vx_pos

        ds1,_ = quad(acc_profile,0,cc_time)
        ds3,_ = quad(dec_profile,0,cc_time)
        ds2 = trajectory[-1].s-ds1-ds3

        dt1 = cc_time
        dt2 = ds2/vx_pos
        dt3 = cc_time

        len_on_traj = 0

        def func_acc(t):
            return (((vx_pos-ego_vx_pos)/cc_time)*t)+ego_vx_pos
        
        def func_dec(t):
            return ((-vx_pos/cc_time)*(t))+vx_pos

        def integral_acc(t):
            integral,err = quad(func_acc,0,t)
            return len_on_traj-integral
        
        def integral_dec(t):
            integral,err = quad(func_dec,0,t)
            return len_on_traj-integral

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
                res = len_on_traj/vx_pos
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
            
            if trajectory[i].t < dt1:
                trajectory[i].vx = round(-func_acc(trajectory[i].t),4)
                i += 1

            if dt1 <= trajectory[i].t <= dt1 + dt2:    
                trajectory[i].vx = round(-vx_pos,4)
                i += 1

            if dt1 + dt2 < trajectory[i].t:
                trajectory[i].vx = round(-func_dec(trajectory[i].t-dt1-dt2),4)
                i += 1
        
        #calculate acceleration values based on velocity derivation

        i=1
        while i < len(trajectory):
            
            if trajectory[i].t < dt1:
                trajectory[i-1].ax = round(derivative(func_acc,trajectory[i-1].t,trajectory[i].t-trajectory[i-1].t),4)
                i += 1

            if dt1 <= trajectory[i].t <= dt1 + dt2:    
                trajectory[i-1].ax = round(0.0,4)
                i += 1

            if dt1 + dt2 < trajectory[i].t:
                trajectory[i-1].ax = round(derivative(func_dec,trajectory[i-1].t,trajectory[i].t-trajectory[i-1].t),4)
                i += 1
        trajectory[-1].ax = 0

    def give_closestprojection_on_trajectory(self,trajectory):

        closest_trajpoint = trajectory[0]

        for trajpoint in trajectory:

            if self.calc_distance_angle_PoseA_PoseB(trajpoint,self.ego_pose) < self.calc_distance_angle_PoseA_PoseB(closest_trajpoint,self.ego_pose):
                closest_trajpoint = trajpoint

        return closest_trajpoint, closest_trajpoint.s


    def give_latprojection_on_trajectory(self,trajectory):

        projection_yaw = np.deg2rad((np.rad2deg(self.ego_pose.yaw)+360+90)%360)

        projection_x1 = self.ego_pose.x
        projection_y1 = self.ego_pose.y 
        projection_x2 = self.ego_pose.x + (np.cos(projection_yaw))
        projection_y2 = self.ego_pose.y + (np.sin(projection_yaw))

        length_on_trajectory = 0

        i = 1

        while i < len(trajectory):

            x,y = self.find_intersection(trajectory[i-1].x,trajectory[i].x,trajectory[i-1].y,trajectory[i].y,\
                                            projection_x1,projection_x2,projection_y1,projection_y2)

            x = round(x,4)
            y = round(y,4)

            if (trajectory[i-1].x <= x <= trajectory[i].x or trajectory[i-1].x >= x >= trajectory[i].x) and\
                (trajectory[i-1].y <= y <= trajectory[i].y or trajectory[i-1].y >= y >= trajectory[i].y):

                dx = abs(x-trajectory[i-1].x)
                dy = abs(y-trajectory[i-1].y)
                hyp = np.hypot(dx, dy)

                length_on_trajectory = trajectory[i-1].s + hyp
                
                return Pose(x,y), length_on_trajectory
            
            i += 1

    def calc_distance_angle_PoseA_PoseB(self,PoseA,PoseB):
        dx = PoseA.x - PoseB.x
        dy = PoseA.y - PoseB.y
        d = np.hypot(dx, dy)
        theta = self.angle_interval(PoseB.yaw-PoseA.yaw)
        return d, theta

    def calc_lin_interpol_angle(self,x1,x2,y1,y2,x3):

        y1 = (np.rad2deg(y1)+360)%360
        y2 = (np.rad2deg(y2)+360)%360

        max_v = max(y1,y2)
        min_v = min(y1,y2)

        propA = max_v-min_v
        propB = 360-propA
        propF = min(propA,propB)
        delta = self.calc_lin_interpol(x1,x2,0,propF,x3)

        if propF == propA:
            interpolated_v = min_v+delta
        elif propF == propB:
            interpolated_v = max_v+delta
        
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

        d = (det((LineA_x1,LineA_y1),(LineA_x2,LineA_y2)),\
            det((LineB_x1, LineB_y1),(LineB_x2,LineB_y2)))

        x = det(d, xdiff) / div
        y = det(d, ydiff) / div

        return x,y

    @staticmethod
    def angle_interval(angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

