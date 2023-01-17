import math
import random
import time
import matplotlib.pyplot as plt
import numpy as np
from statemachine import StateMachine, State
import pyclothoids

show_animation = True

class G2ClothoidPlanner:

    trajectory = []
    temptrajectory23 = []

    ego_pose = None
    goal_pose = None
    
    def __init__(self, path_res = 0.1,path23_res = 0.1):
        self.path_res = path_res
        self.path23_res = path23_res

    def update_ego_pose(self,ego_pose):
        self.ego_pose = ego_pose
        return

    def update_goal_pose(self,goal_pose):
        self.goal_pose = goal_pose
        return

    def generate_trajectory(self):

        g2clothoid_list = pyclothoids.SolveG2(self.ego_pose.x, self.ego_pose.y, self.ego_pose.yaw, 0,\
                                                self.goal_pose.x, self.goal_pose.y, self.goal_pose.yaw, 0)
        g2clothoid_trajectory = self.sample_trajectory(g2clothoid_list)

        self.trajectory = g2clothoid_trajectory
    
        return self.trajectory

    def sample_trajectory(self, g2clothoid_list):

        total_length = g2clothoid_list[0].length + g2clothoid_list[1].length + g2clothoid_list[2].length
        npt_tar = round(total_length/self.path_res)
        samp_int = total_length/npt_tar

        trajectory_point_list = []
        samp_point = 0
        
        while samp_point < g2clothoid_list[0].length:

            trajectory_point_list.append(TrajectoryPoint(round(samp_point,4),
                                                            round(g2clothoid_list[0].X(samp_point),4),
                                                            round(g2clothoid_list[0].Y(samp_point),4),
                                                            round(g2clothoid_list[0].Theta(samp_point),4),
                                                            round(g2clothoid_list[0].KappaStart + (g2clothoid_list[0].dk*samp_point)),4))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length < g2clothoid_list[1].length:

            trajectory_point_list.append(TrajectoryPoint(round(samp_point,4),
                                                            round(g2clothoid_list[1].X(samp_point - g2clothoid_list[0].length),4),
                                                            round(g2clothoid_list[1].Y(samp_point - g2clothoid_list[0].length),4),
                                                            round(g2clothoid_list[1].Theta(samp_point - g2clothoid_list[0].length),4),
                                                            round(g2clothoid_list[1].KappaStart + (g2clothoid_list[1].dk*(samp_point-g2clothoid_list[0].length))),4))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length < g2clothoid_list[2].length:
            
            trajectory_point_list.append(TrajectoryPoint(round(samp_point,4),
                                                            round(g2clothoid_list[2].X(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                            round(g2clothoid_list[2].Y(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                            round(g2clothoid_list[2].Theta(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),4),
                                                            round(g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length))),4))

            samp_point += samp_int


        trajectory_point_list.append(TrajectoryPoint(round(total_length,4),
                                                        round(g2clothoid_list[2].X(g2clothoid_list[2].length),4),
                                                        round(g2clothoid_list[2].Y(g2clothoid_list[2].length),4),
                                                        round(g2clothoid_list[2].Theta(g2clothoid_list[2].length),4),
                                                        round(g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*g2clothoid_list[2].length)),4))

        
        trajectory_point_list.reverse()
        x = 0

        for trajectory_point in trajectory_point_list:

            trajectory_point.velocity = round(1/(1+math.exp((-4*x)+2)),3)

            x += samp_int

        trajectory_point_list.reverse()

        return trajectory_point_list

    def give_closestprojection_on_trajectory(self):

        closest_trajpoint = self.trajectory[0]

        for trajpoint in self.trajectory:

            if self.calc_distance_angle_PoseA_PoseB(trajpoint,self.ego_pose) < self.calc_distance_angle_PoseA_PoseB(closest_trajpoint,self.ego_pose):
                closest_trajpoint = trajpoint

        return closest_trajpoint, closest_trajpoint.length


    def give_latprojection_on_trajectory(self):

        projection_yaw = np.deg2rad((np.rad2deg(self.ego_pose.yaw)+360+90)%360)

        projection_x1 = self.ego_pose.x
        projection_y1 = self.ego_pose.y 
        projection_x2 = self.ego_pose.x + (np.cos(projection_yaw))
        projection_y2 = self.ego_pose.y + (np.sin(projection_yaw))

        length_on_trajectory = 0

        i = 1

        while i < len(self.trajectory):

            x,y = self.find_intersection(self.trajectory[i-1].x,self.trajectory[i].x,self.trajectory[i-1].y,self.trajectory[i].y,\
                                            projection_x1,projection_x2,projection_y1,projection_y2)

            x = round(x,4)
            y = round(y,4)

            if (self.trajectory[i-1].x <= x <= self.trajectory[i].x or self.trajectory[i-1].x >= x >= self.trajectory[i].x) and\
                (self.trajectory[i-1].y <= y <= self.trajectory[i].y or self.trajectory[i-1].y >= y >= self.trajectory[i].y):

                dx = abs(x-self.trajectory[i-1].x)
                dy = abs(y-self.trajectory[i-1].y)
                hyp = math.hypot(dx, dy)

                length_on_trajectory = self.trajectory[i-1].length + hyp
                
                return Pose(x,y), length_on_trajectory
            
            i += 1

    def resample_trajectory23(self):

        #_,length_on_trajectory = self.give_latprojection_on_trajectory()
        _,length_on_trajectory = self.give_closestprojection_on_trajectory()

        trajectory23 = []
        j = 1
        trajectory23_cnt = 0

        while j < len(self.trajectory) and trajectory23_cnt < 23:
            
            if self.trajectory[j-1].length <= length_on_trajectory < self.trajectory[j].length:

                new_traj_point = TrajectoryPoint(length_on_trajectory-self.trajectory[-1].length)

                new_traj_point.x = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j].length,\
                                                            self.trajectory[j-1].x,self.trajectory[j].x,length_on_trajectory)

                new_traj_point.y = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j].length,\
                                                            self.trajectory[j-1].y,self.trajectory[j].y,length_on_trajectory)
                
                new_traj_point.yaw = np.deg2rad(self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j].length,\
                                                            (np.rad2deg(self.trajectory[j-1].yaw)+360)%360,(np.rad2deg(self.trajectory[j].yaw)+360)%360,length_on_trajectory))

                new_traj_point.curvature = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j].length,\
                                                            self.trajectory[j-1].curvature,self.trajectory[j].curvature,length_on_trajectory)

                new_traj_point.velocity = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j].length,\
                                                            self.trajectory[j-1].velocity,self.trajectory[j].velocity,length_on_trajectory)

                trajectory23.append(new_traj_point)

                length_on_trajectory += self.path23_res
                trajectory23_cnt += 1
                j = 0
                
            j += 1
            self.temptrajectory23 = trajectory23
            
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
    def calc_distance_angle_PoseA_PoseB(PoseA,PoseB):
        dx = PoseA.x - PoseB.x
        dy = PoseA.y - PoseB.y
        d = math.hypot(dx, dy)
        theta = PoseA.yaw - PoseB.yaw
        return d, theta

class PlannerMachine(StateMachine):

    path_res = 0.1
    path23_res = 0.2
    ego_delta_bilevel = 0.5
    goal_delta_bilevel = 0.15
    max_curvature = 0.5
    min_traj_length = 2

    planner = G2ClothoidPlanner(path_res,path23_res)

    #state_declaration
    init = State('state', initial=True)
    goalreached_check = State('goalreached_check')
    goalreached = State('goalreached')
    gen_trajectory = State('gen_trajectory')
    bilevel_check = State('bilevel_check')
    feasibility_check = State('feasibility_check')
    send_trajectory = State('send_trajectory')
    error = State('error')
    
    #transition_declaration
    initialization = init.to(goalreached_check)
    goalreached_check_success = goalreached_check.to(goalreached)
    goalreached_check_fail = goalreached_check.to(bilevel_check)
    goalreached_reset = goalreached.to(init)
    trajectory_generated_success = gen_trajectory.to(bilevel_check)
    bilevel_check_fail = bilevel_check.to(gen_trajectory)
    bilevel_check_success = bilevel_check.to(feasibility_check)
    feasibility_check_fail = feasibility_check.to(error)
    feasibility_check_success = feasibility_check.to(send_trajectory)
    send_trajectory_reset = send_trajectory.to(init)
    error_reset = error.to(init)

    def on_enter_goalreached_check(self):
        print('State: goalreached_check',end= " ")

        d, theta = self.planner.calc_distance_angle_PoseA_PoseB(self.planner.ego_pose,self.planner.goal_pose)

        if d <= 0.3: #and theta <= 0.1:
            print("success")
            self.goalreached_check_success()
        
        else:
            print("failed: goaldistance - ",d)
            self.goalreached_check_fail()

    def on_enter_goalreached(self):
        print('State: goalreached')

        self.goalreached_reset()
    
    def on_enter_gen_trajectory(self):
        print('State: gen_trajectory',end= " ")

        self.planner.generate_trajectory()
        print("success")

        self.trajectory_generated_success()

    def on_enter_bilevel_check(self):
        print("State: bilevel_check",end= " ")

        if self.planner.trajectory:
        
            dis_goal_trajgoal,theta_goal_trajgoal = self.planner.calc_distance_angle_PoseA_PoseB(self.planner.trajectory[-1],self.planner.goal_pose)

            point_on_trajectory,_ = self.planner.give_closestprojection_on_trajectory()
            dis_ego_traj,theta_ego_traj = self.planner.calc_distance_angle_PoseA_PoseB(point_on_trajectory,self.planner.ego_pose)

            #---TransitionCondition---#
            if dis_goal_trajgoal <= self.goal_delta_bilevel:
                if dis_ego_traj <= self.ego_delta_bilevel:
                    print("success")
                    self.bilevel_check_success()
                else:
                    print("failed: ego Pose not on trajectory")
                    self.bilevel_check_fail()
            else:   
                print("failed: goal not on trajectory")
                self.bilevel_check_fail()
        else:
            print("failed: not found trajectory")
            self.bilevel_check_fail()

    def on_enter_feasibility_check(self):
        print("State: feasibility_check",end= " ")

        curvature_feasible = True

        for trajectory_point in self.planner.trajectory:
            if trajectory_point.curvature > self.max_curvature:
                curvature_feasible = False

        #---TransitionCondition---#
        if self.planner.trajectory[-1].length > self.min_traj_length:
            if curvature_feasible:
                print("success")
                self.feasibility_check_success()
            else:
                print("failed: curvature not feasible")
                self.feasibility_check_fail()
        else:
            print("failed: trajectory too short")
            self.feasibility_check_fail()

    def on_enter_send_trajectory(self):
        print('State: send_trajectory')

        self.planner.resample_trajectory23()

        self.send_trajectory_reset()
    
    def on_enter_error(self):
        print('State: error')

        self.error_reset()


class TrajectoryPoint:

    def __init__(self,length,x=None,y=None,yaw=None,curvature=None,velocity=None):
        self.length = length
        self.x = x   
        self.y = y
        self.yaw = yaw
        self.curvature = curvature
        self.velocity = velocity

class Pose:

    def __init__(self, x, y, yaw=None, curvature=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curvature = curvature

class Simulation:

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self, init_pose, goal_pose,play_area):
        self.init_pose = Pose(init_pose[0],init_pose[1],init_pose[2], init_pose[3])
        self.goal_pose = Pose(goal_pose[0],goal_pose[1],goal_pose[2], goal_pose[3])
        self.ego = self.init_pose
        self.plannermachine = PlannerMachine()

        self.ego_on_trajectorylength = 0
        self.drive_step = 0.3

        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None

    def ego_drive_step(self):

        _, self.ego_on_trajectorylength = self.plannermachine.planner.give_latprojection_on_trajectory()

        self.ego_on_trajectorylength += self.drive_step

        i = 1
        
        while i < len(self.plannermachine.planner.trajectory):

            if self.plannermachine.planner.trajectory[i-1].length <= self.ego_on_trajectorylength < self.plannermachine.planner.trajectory[i].length:
                

                
                x = self.plannermachine.planner.calc_lin_interpol(self.plannermachine.planner.trajectory[i-1].length,self.plannermachine.planner.trajectory[i].length,\
                                                self.plannermachine.planner.trajectory[i-1].x,self.plannermachine.planner.trajectory[i].x,self.ego_on_trajectorylength)

                y = self.plannermachine.planner.calc_lin_interpol(self.plannermachine.planner.trajectory[i-1].length,self.plannermachine.planner.trajectory[i].length,\
                                                self.plannermachine.planner.trajectory[i-1].y,self.plannermachine.planner.trajectory[i].y,self.ego_on_trajectorylength)
                
                yaw = np.deg2rad(self.plannermachine.planner.calc_lin_interpol(self.plannermachine.planner.trajectory[i-1].length,self.plannermachine.planner.trajectory[i].length,\
                                                (np.rad2deg(self.plannermachine.planner.trajectory[i-1].yaw)+360)%360,(np.rad2deg(self.plannermachine.planner.trajectory[i].yaw)+360)%360,self.ego_on_trajectorylength))

                self.ego.x = x + np.random.normal(0,0.1)
                self.ego.y = y + np.random.normal(0,0.1)
                self.ego.yaw = yaw + np.random.normal(0,0.01)
                
                break
        
            i += 1
    
    def simulate(self):
    
        counter = 0
        
        while True:

            print("cycle: ", counter)
            counter += 1

            self.plannermachine.planner.update_ego_pose(self.ego)
            self.plannermachine.planner.update_goal_pose(self.goal_pose)

            self.plannermachine.initialization()
            
            self.visualization()

            self.ego_drive_step()

            time.sleep(0.5)
        
    def visualization(self, rnd = None):
        plt.figure(1)
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',lambda event: [exit(0) if event.key == 'escape' else None])

        
        if self.plannermachine.planner.trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.plannermachine.planner.trajectory:   
                x += [trajectory_point.x]
                y += [trajectory_point.y]
            
            plt.plot(x,y, "-g")
        
        if self.plannermachine.planner.temptrajectory23 is not None:
            
            x = []
            y = []

            for trajectory_point in self.plannermachine.planner.temptrajectory23:   
                x += [trajectory_point.x]
                y += [trajectory_point.y]
            
            plt.plot(x,y, "-r")

        if self.ego is not None:
            self.plot_arrow(self.ego.x,self.ego.y, self.ego.yaw)

        if self.init_pose is not None:
            self.plot_arrow(self.init_pose.x,self.init_pose.y,self.init_pose.yaw,0.75,0.75,"r")

        if self.goal_pose is not None:
            self.plot_arrow(self.goal_pose.x,self.goal_pose.y,self.goal_pose.yaw,0.75,0.75,"r")

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.axis([-2, 22, -2, 22])
        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)

    def LinePlot(self):

        plt.figure(2)
        plt.clf()

        if self.init_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.init_trajectory:   
                x += [trajectory_point.length]
                y += [trajectory_point.curvature]
            

            plt.plot(x,y, "-b")
        
        if self.driven_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.driven_trajectory:   
                x += [trajectory_point.length]
                y += [trajectory_point.curvature]
            

            plt.plot(x,y, "-r")

        if self.planner.trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.planner.trajectory:   
                x += [trajectory_point.length + self.ego.arc_length]
                y += [trajectory_point.curvature]
            
            plt.plot(x,y, "-g")
        
        plt.grid(True)
        
        plt.figure(3)
        plt.clf()

        if self.init_trajectory is not None:
            
            x = []
            y = []

            for trajectory_point in self.init_trajectory:   
                x += [trajectory_point.length]
                y += [trajectory_point.velocity]
            
            plt.plot(x,y, "-g")

            plt.grid(True)
            plt.axis("equal")



    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok
    
    @staticmethod
    def plot_arrow(x, y, yaw, length=0.75, width=0.75, fc="g", ec="k"):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

    
   
def main():
    print("start " + __file__)
    simulation = Simulation(goal_pose=[12,13, np.deg2rad(90),0],
                            init_pose=[3,5, np.deg2rad(20),0],
                            play_area=[0,20,0,20])

    simulation.simulate()

if __name__ == '__main__':
    main()