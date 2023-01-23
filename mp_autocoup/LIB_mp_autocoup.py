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

    ego_pose = Pose()
    goal_pose = Pose()

    ego_on_trajectorylength = 0
    drive_step = 0.1
    
    def __init__(self, path_res=0.1, path23_res=0.1, vx=1.5, ego_delta_bilevel=0.5, goal_delta_bilevel=0.15, max_curvature=0.5, min_traj_length=2):
        self.path_res = path_res
        self.path23_res = path23_res
        self.vx = vx

        self.ego_delta_bilevel = ego_delta_bilevel
        self.goal_delta_bilevel = goal_delta_bilevel

        self.max_curvature = max_curvature
        self.min_traj_length = min_traj_length

        self.trajectory_p1 = []
        self.trajectory_p2 = []

        self.trajectory23 = []

    def update_ego_pose_reverse(self):
        return self.ego_pose

    def update_ego_pose(self, ego_pose):
        self.ego_pose = ego_pose

    def update_goal_pose(self,goal_pose):
        self.goal_pose = goal_pose

    def cycle(self):
        
        if self.goal_not_reached():

            self.bilevel_check()

            if self.feasibility_check():
                self.resample_trajectory23()
                return self.trajectory23
            else:
                print("Abort Mission: not feasible or too short")
        else: 
            print("goal reacheed")

    def goal_not_reached(self):

        distance_ego_goal, theta_ego_goal = self.calc_distance_angle_PoseA_PoseB(self.ego_pose,self.goal_pose)

        if distance_ego_goal > 0.3:
            return True
        else:
            return False

    def bilevel_check(self):

        if not self.trajectory_p1:
            self.generate_trajectory()

        dis_goal_trajgoal, theta_goal_trajgoal = self.calc_distance_angle_PoseA_PoseB(self.trajectory_p1[-1],self.goal_pose)
        point_on_trajectory,_ = self.give_closestprojection_on_trajectory()
        dis_ego_traj,theta_ego_traj = self.calc_distance_angle_PoseA_PoseB(point_on_trajectory,self.ego_pose)

        if dis_goal_trajgoal > self.goal_delta_bilevel and dis_ego_traj > self.ego_delta_bilevel:
            print("failed: ego or goal not on trajectory")
            self.generate_trajectory()

    def feasibility_check(self):

        curvature_feasible = True
        for trajectory_point in self.trajectory_p1:
            if trajectory_point.curvature > self.max_curvature:
                curvature_feasible = False

        if self.trajectory_p1[-1].s > self.min_traj_length and curvature_feasible:
            return True
        else:
            return False
        
    def ego_drive_step(self):

        _, self.ego_on_trajectorylength = self.give_closestprojection_on_trajectory()

        self.ego_on_trajectorylength += self.drive_step

        i = 1
        
        while i < len(self.trajectory_p1):

            if self.trajectory_p1[i-1].s <= self.ego_on_trajectorylength < self.trajectory_p1[i].s:
                
                x = self.calc_lin_interpol(self.trajectory_p1[i-1].s,self.trajectory_p1[i].s,\
                                            self.trajectory_p1[i-1].x,self.trajectory_p1[i].x,self.ego_on_trajectorylength)

                y = self.calc_lin_interpol(self.trajectory_p1[i-1].s,self.trajectory_p1[i].s,\
                                            self.trajectory_p1[i-1].y,self.trajectory_p1[i].y,self.ego_on_trajectorylength)
                
                yaw =self.calc_lin_interpol_angle(self.trajectory_p1[i-1].s,self.trajectory_p1[i].s,\
                                                    self.trajectory_p1[i-1].yaw,self.trajectory_p1[i].yaw,self.ego_on_trajectorylength)

                self.ego_pose.x = x + np.random.normal(0,0.1)
                self.ego_pose.y = y + np.random.normal(0,0.1)
                self.ego_pose.yaw = yaw + np.random.normal(0,0.01)
                
                break
        
            i += 1

    def generate_trajectory(self):

        g2clothoid_list = pyclothoids.SolveG2(self.ego_pose.x, self.ego_pose.y, self.angle_interval(self.ego_pose.yaw+np.pi), self.ego_pose.curvature,\
                                                self.goal_pose.x, self.goal_pose.y, self.angle_interval(self.goal_pose.yaw+np.pi), self.goal_pose.curvature)
        
        self.sample_trajectory_path(g2clothoid_list)

        self.offset_yaw()
        self.add_vx2trajectory()
        self.add_timestamp2trajectory()
        self.add_ax2trajectory()

    def sample_trajectory_path(self, g2clothoid_list):

        self.trajectory_p1.clear()

        total_length = g2clothoid_list[0].length + g2clothoid_list[1].length + g2clothoid_list[2].length
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

        
        self.trajectory_p1.append(TrajectoryPoint( s=round(total_length,4),
                                                x=round(g2clothoid_list[2].X(g2clothoid_list[2].length),4),
                                                y=round(g2clothoid_list[2].Y(g2clothoid_list[2].length),4),
                                                yaw=round(g2clothoid_list[2].Theta(g2clothoid_list[2].length),4),
                                                curvature=round((g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*g2clothoid_list[2].length)),4)))
        

    def resample_trajectory23(self):

        self.trajectory23.clear()

        #_,length_on_trajectory = self.give_latprojection_on_trajectory()
        _,length_on_trajectory = self.give_closestprojection_on_trajectory()

        j = 1
        trajectory23_cnt = 0

        length_offset = length_on_trajectory
        time_offset = self.calc_lin_interpol(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                self.trajectory_p1[j-1].t,self.trajectory_p1[j].t,length_on_trajectory)

        while j < len(self.trajectory_p1) and trajectory23_cnt < 23:
            
            if self.trajectory_p1[j-1].s <= length_on_trajectory < self.trajectory_p1[j].s:

                new_traj_point = TrajectoryPoint(s=(self.path23_res*trajectory23_cnt))

                new_traj_point.t = (self.calc_lin_interpol(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                            self.trajectory_p1[j-1].t,self.trajectory_p1[j].t,length_on_trajectory) - time_offset)

                new_traj_point.x = self.calc_lin_interpol(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                            self.trajectory_p1[j-1].x,self.trajectory_p1[j].x,length_on_trajectory)

                new_traj_point.y = self.calc_lin_interpol(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                            self.trajectory_p1[j-1].y,self.trajectory_p1[j].y,length_on_trajectory)
                
                new_traj_point.yaw = self.calc_lin_interpol_angle(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                                    self.trajectory_p1[j-1].yaw,self.trajectory_p1[j].yaw,length_on_trajectory)

                new_traj_point.curvature = self.calc_lin_interpol(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                            self.trajectory_p1[j-1].curvature,self.trajectory_p1[j].curvature,length_on_trajectory)

                new_traj_point.vx = self.calc_lin_interpol(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                            self.trajectory_p1[j-1].vx,self.trajectory_p1[j].vx,length_on_trajectory)
                
                new_traj_point.ax = self.calc_lin_interpol(self.trajectory_p1[j-1].s,self.trajectory_p1[j].s,\
                                                            self.trajectory_p1[j-1].ax,self.trajectory_p1[j].ax,length_on_trajectory)

                self.trajectory23.append(new_traj_point)

                length_on_trajectory += self.path23_res
                trajectory23_cnt += 1
                j = 0
                
            j += 1

    def offset_yaw(self):

        for trajectory_point in self.trajectory_p1:
            trajectory_point.yaw = self.angle_interval(trajectory_point.yaw-np.pi)

    def add_vx2trajectory(self):

        self.trajectory_p1.reverse()

        braking_dis = 0.5
        m=self.vx/braking_dis

        for trajectory_point in self.trajectory_p1:

            if abs(trajectory_point.s - self.trajectory_p1[0].s) <= braking_dis:

                trajectory_point.vx = round((m*abs(trajectory_point.s - self.trajectory_p1[0].s)),4)
            else:
                trajectory_point.vx = self.vx

        self.trajectory_p1.reverse()

    def add_timestamp2trajectory(self):

        i = 1

        sum = 0
        self.trajectory_p1[0].t = sum
        
        while i < len(self.trajectory_p1):

            sum += self.trapez_integral(self.divide_ZE(1,self.trajectory_p1[i-1].vx),self.divide_ZE(1,self.trajectory_p1[i].vx),self.trajectory_p1[i-1].s,self.trajectory_p1[i].s)
            self.trajectory_p1[i].t = sum 
        
            i += 1

    def add_ax2trajectory(self):

        i = 1

        while i < len(self.trajectory_p1):

            self.trajectory_p1[i-1].ax = (self.trajectory_p1[i].vx-self.trajectory_p1[i-1].vx)/(self.trajectory_p1[i].t-self.trajectory_p1[i-1].t)

            i += 1
            
        self.trajectory_p1[-1].ax = 0

    def give_closestprojection_on_trajectory(self):

        closest_trajpoint = self.trajectory_p1[0]

        for trajpoint in self.trajectory_p1:

            if self.calc_distance_angle_PoseA_PoseB(trajpoint,self.ego_pose) < self.calc_distance_angle_PoseA_PoseB(closest_trajpoint,self.ego_pose):
                closest_trajpoint = trajpoint

        return closest_trajpoint, closest_trajpoint.s


    def give_latprojection_on_trajectory(self):

        projection_yaw = np.deg2rad((np.rad2deg(self.ego_pose.yaw)+360+90)%360)

        projection_x1 = self.ego_pose.x
        projection_y1 = self.ego_pose.y 
        projection_x2 = self.ego_pose.x + (np.cos(projection_yaw))
        projection_y2 = self.ego_pose.y + (np.sin(projection_yaw))

        length_on_trajectory = 0

        i = 1

        while i < len(self.trajectory_p1):

            x,y = self.find_intersection(self.trajectory_p1[i-1].x,self.trajectory_p1[i].x,self.trajectory_p1[i-1].y,self.trajectory_p1[i].y,\
                                            projection_x1,projection_x2,projection_y1,projection_y2)

            x = round(x,4)
            y = round(y,4)

            if (self.trajectory_p1[i-1].x <= x <= self.trajectory_p1[i].x or self.trajectory_p1[i-1].x >= x >= self.trajectory_p1[i].x) and\
                (self.trajectory_p1[i-1].y <= y <= self.trajectory_p1[i].y or self.trajectory_p1[i-1].y >= y >= self.trajectory_p1[i].y):

                dx = abs(x-self.trajectory_p1[i-1].x)
                dy = abs(y-self.trajectory_p1[i-1].y)
                hyp = np.hypot(dx, dy)

                length_on_trajectory = self.trajectory_p1[i-1].s + hyp
                
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
        try:
            return x/y
        except ZeroDivisionError:
            return 0

    @staticmethod
    def angle_interval(angle):
        return (angle + np.pi) % (2*np.pi) - np.pi

