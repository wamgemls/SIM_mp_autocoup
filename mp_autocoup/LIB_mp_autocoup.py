import numpy as np
import pyclothoids

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

class AutocoupPlanner:

    ego_on_trajectorylength = 0
    drive_step = 0.2
    
    def __init__(self, path_res=0.1, path23_res=0.1, vx=-0.41, ego_delta_bilevel=0.5, goal_delta_bilevel=0.15, max_curvature=0.26, min_traj_length=2):
        self.path_res = path_res
        self.path23_res = path23_res
        self.vx = vx

        self.ego_delta_bilevel = ego_delta_bilevel
        self.goal_delta_bilevel = goal_delta_bilevel

        self.max_curvature = max_curvature
        self.min_traj_length = min_traj_length

        self.dis_prekingpin_kingpin = 2
        self.linear_backup = 1

        self.init_pose = Pose()
        self.ego_pose = Pose()
        self.prekingpin_pose = Pose()
        self.kingpin_pose = Pose()

        self.trajectory_p1 = []
        self.trajectory_p2 = []

        self.trajectory23 = []

    def update_pose_reverse(self):
        return self.ego_pose,self.prekingpin_pose

    def update_pose(self, init_pose, ego_pose, kingpin_pose):
        self.init_pose = init_pose
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

            if self.coupling_phase_2():
                print("phase_2")
                
                self.resample_trajectory23(self.trajectory_p2)
                return self.trajectory23

            elif self.coupling_phase_1():
                print("phase_1")
                
                self.bilevel_check()
                
                if self.feasibility_check():
                    
                    self.resample_trajectory23(self.trajectory_p1)
                    return self.trajectory23
                else:
                    print("abort_mission")   
            else:
                print("invalid_phase")
        else:
            print("invalid_pose")

    def ego_drive_step(self):

        _, self.ego_on_trajectorylength = self.give_closestprojection_on_trajectory(self.trajectory23)

        self.ego_on_trajectorylength += self.drive_step

        i = 1
        
        while i < len(self.trajectory23):

            if self.trajectory23[i-1].s <= self.ego_on_trajectorylength < self.trajectory23[i].s:
                
                x = self.calc_lin_interpol(self.trajectory23[i-1].s,self.trajectory23[i].s,\
                                            self.trajectory23[i-1].x,self.trajectory23[i].x,self.ego_on_trajectorylength)

                y = self.calc_lin_interpol(self.trajectory23[i-1].s,self.trajectory23[i].s,\
                                            self.trajectory23[i-1].y,self.trajectory23[i].y,self.ego_on_trajectorylength)
                
                yaw =self.calc_lin_interpol_angle(self.trajectory23[i-1].s,self.trajectory23[i].s,\
                                                    self.trajectory23[i-1].yaw,self.trajectory23[i].yaw,self.ego_on_trajectorylength)

                self.ego_pose.x = x + np.random.normal(0,0.1)
                self.ego_pose.y = y + np.random.normal(0,0.1)
                self.ego_pose.yaw = yaw #+ np.random.normal(0,0.01)
                
                break
        
            i += 1

    def pose_validation(self):
        return True

    def coupling_phase_1(self):

        dis_ego_kingpin,_ = self.calc_distance_angle_PoseA_PoseB(self.ego_pose,self.kingpin_pose)

        if dis_ego_kingpin > self.dis_prekingpin_kingpin:

            if not self.trajectory_p1 or not self.trajectory_p2:
                self.sample_trajectory_p1()
                self.sample_trajectory_p2
            return True
        else:
            return False

    def coupling_phase_2(self):

        if not self.trajectory_p2:
            self.sample_trajectory_p2()

        point_on_trajectory,_ = self.give_closestprojection_on_trajectory(self.trajectory_p2)
        dis_ego_traj,theta_ego_traj = self.calc_distance_angle_PoseA_PoseB(point_on_trajectory,self.ego_pose)

        if dis_ego_traj < 0.3 and theta_ego_traj < 0.1:
            return True
        else:
            return False

    def bilevel_check(self):

        dis_goal_trajgoal, theta_goal_trajgoal = self.calc_distance_angle_PoseA_PoseB(self.trajectory_p1[-1],self.prekingpin_pose)
        point_on_trajectory,_ = self.give_closestprojection_on_trajectory(self.trajectory_p1)
        dis_ego_traj,theta_ego_traj = self.calc_distance_angle_PoseA_PoseB(point_on_trajectory,self.ego_pose)

        if dis_goal_trajgoal > self.goal_delta_bilevel and dis_ego_traj > self.ego_delta_bilevel:
            print("failed: ego or goal not on trajectory")
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
        preprekingpin_calc_x = self.prekingpin_pose.x - (self.linear_backup*np.cos(preprekingpin_calc_angel))
        preprekingpin_calc_y = self.prekingpin_pose.y - (self.linear_backup*np.sin(preprekingpin_calc_angel))

        g2clothoid_list = pyclothoids.SolveG2(self.ego_pose.x, self.ego_pose.y, ego_calc_angle, self.ego_pose.curvature,\
                                                preprekingpin_calc_x, preprekingpin_calc_y, preprekingpin_calc_angel, 0)
        
        #self.sample_trajectory_p1_path(g2clothoid_list)

        total_length = g2clothoid_list[0].length + g2clothoid_list[1].length + g2clothoid_list[2].length + self.linear_backup
        npt_tar = round(total_length/self.path_res)
        samp_int = total_length/npt_tar

        samp_point = 0
        
        while samp_point <= g2clothoid_list[0].length and len(self.trajectory_p1) <= npt_tar:

            self.trajectory_p1.append(TrajectoryPoint( s=round(samp_point,4),
                                                    x=round(g2clothoid_list[0].X(samp_point),4),
                                                    y=round(g2clothoid_list[0].Y(samp_point),4),
                                                    yaw=round(g2clothoid_list[0].Theta(samp_point),4),
                                                    curvature=round((g2clothoid_list[0].KappaStart + (g2clothoid_list[0].dk*samp_point)),4)))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length <= g2clothoid_list[1].length and len(self.trajectory_p1) <= npt_tar:

            self.trajectory_p1.append(TrajectoryPoint( s=round(samp_point,4),
                                                    x=round(g2clothoid_list[1].X(samp_point - g2clothoid_list[0].length),4),
                                                    y=round(g2clothoid_list[1].Y(samp_point - g2clothoid_list[0].length),4),
                                                    yaw=round(g2clothoid_list[1].Theta(samp_point - g2clothoid_list[0].length),4),
                                                    curvature=round((g2clothoid_list[1].KappaStart + (g2clothoid_list[1].dk*(samp_point-g2clothoid_list[0].length))),4)))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length <= g2clothoid_list[2].length and len(self.trajectory_p1) <= npt_tar:
            
            self.trajectory_p1.append(TrajectoryPoint( s=round(samp_point,4),
                                                    x=round(g2clothoid_list[2].X(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                    y=round(g2clothoid_list[2].Y(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                    yaw=round(g2clothoid_list[2].Theta(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                    curvature=round((g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length))),4)))

            samp_point += samp_int

        
        linear_length = samp_int
        
        while samp_point < total_length and len(self.trajectory_p1) <= npt_tar:

            self.trajectory_p1.append(TrajectoryPoint(s=round(samp_point,4),
                                                        x = round(preprekingpin_calc_x + (linear_length * np.cos(preprekingpin_calc_angel)),4),
                                                        y = round(preprekingpin_calc_y + (linear_length * np.sin(preprekingpin_calc_angel)),4),
                                                        yaw = round(preprekingpin_calc_angel,4),
                                                        curvature=0))

            linear_length += samp_int
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

            self.trajectory_p2.append(TrajectoryPoint(s=round(samp_point,4),
                                                        x = round(self.prekingpin_pose.x + (samp_point * np.cos(traj_yaw)),4),
                                                        y = round(self.prekingpin_pose.y + (samp_point * np.sin(traj_yaw)),4),
                                                        yaw = round(traj_yaw,4),
                                                        curvature=0))

            samp_point += samp_int

        self.offset_yaw(self.trajectory_p2)
        self.add_long2trajectory(self.trajectory_p2)

    def resample_trajectory23(self,trajectory):

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

        acc_dece_dis = 1
        m_acceleration = (self.vx-self.ego_pose.vx)/acc_dece_dis
        m_deceleration = -self.vx/acc_dece_dis

        i=0
        trajectory[0].t= 0

        while i < len(trajectory):

            if trajectory[i].s <= acc_dece_dis:

                trajectory[i].vx = m_acceleration*trajectory[i].s+self.ego_pose.vx

                if i > 0:
                    trajectory[i].t = abs((1/(2*m_acceleration))*(trajectory[i].s**2-trajectory[i-1].s**2))

            elif trajectory[i].s >= trajectory[-1].s - acc_dece_dis:

                trajectory[i].vx = m_deceleration*(trajectory[i].s-trajectory[-1].s)

                if i > 0:
                    trajectory[i].t = abs((1/(2*m_deceleration))*(trajectory[i].s**2-trajectory[i-1].s**2))

            else:
                trajectory[i].vx = self.vx

                if i > 0:
                    trajectory[i].t = abs(trajectory[i].s/self.vx)

            if i > 0:
                trajectory[i-1].ax = (trajectory[i].vx-trajectory[i-1].vx)/(trajectory[i].t-trajectory[i-1].t)
            
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
    def trapez_integral(fa,fb,a,b):
        return ((b-a)/2) * (fa+fb)

    @staticmethod
    def divide_ZE(x,y):
        if y==0:
            return np.inf
        elif y==-0:
            return -np.inf
        else:
            return x/y

    @staticmethod
    def angle_interval(angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

