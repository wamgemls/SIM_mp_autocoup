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

    ego_pose = None
    goal_pose = None
    
    def __init__(self, pathres = 0.1):
        self.pathres = pathres

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
        npt_tar = round(total_length/self.pathres)
        samp_int = total_length/npt_tar

        trajectory_point_list = []
        samp_point = 0
        
        while samp_point < g2clothoid_list[0].length:

            trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                        g2clothoid_list[0].X(samp_point),
                                                        g2clothoid_list[0].Y(samp_point),
                                                        g2clothoid_list[0].Theta(samp_point),
                                                        g2clothoid_list[0].KappaStart + (g2clothoid_list[0].dk*samp_point)))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length < g2clothoid_list[1].length:

            trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                        g2clothoid_list[1].X(samp_point - g2clothoid_list[0].length),
                                                        g2clothoid_list[1].Y(samp_point - g2clothoid_list[0].length),
                                                        g2clothoid_list[1].Theta(samp_point - g2clothoid_list[0].length),
                                                        g2clothoid_list[1].KappaStart + (g2clothoid_list[1].dk*(samp_point-g2clothoid_list[0].length))))

            samp_point += samp_int

        while samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length < g2clothoid_list[2].length:
            
            trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                        g2clothoid_list[2].X(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),
                                                        g2clothoid_list[2].Y(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),
                                                        g2clothoid_list[2].Theta(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length),
                                                        g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*(samp_point-g2clothoid_list[0].length-g2clothoid_list[1].length))))

            samp_point += samp_int


        trajectory_point_list.append(TrajectoryPoint(total_length,
                                                    g2clothoid_list[2].X(g2clothoid_list[2].length),
                                                    g2clothoid_list[2].Y(g2clothoid_list[2].length),
                                                    g2clothoid_list[2].Theta(g2clothoid_list[2].length),
                                                    g2clothoid_list[2].KappaStart + (g2clothoid_list[2].dk*g2clothoid_list[2].length)))

        
        trajectory_point_list.reverse()
        x = 0

        for trajectory_point in trajectory_point_list:

            trajectory_point.velocity = round(1/(1+math.exp((-4*x)+2)),3)

            x += samp_int

        trajectory_point_list.reverse()

        return trajectory_point_list


    def give_projection_point_on_trajectory(self):

        projection_yaw = np.deg2rad((np.rad2deg(self.ego_pose.yaw)+360+90)%360)

        projection_x1 = self.ego_pose.x
        projection_y1 = self.ego_pose.y 
        projection_x2 = self.ego_pose.x + (np.cos(projection_yaw))
        projection_y2 = self.ego_pose.y + (np.sin(projection_yaw))

        i = 1

        while i < len(self.trajectory):

            x,y = self.find_intersection(self.trajectory[i-1].x,self.trajectory[i].x,self.trajectory[i-1].y,self.trajectory[i].y,\
                                            projection_x1,projection_x2,projection_y1,projection_y2)

            x = round(x,4)
            y = round(y,4)
            print(i, "intersection found")

            if (self.trajectory[i-1].x <= x < self.trajectory[i].x or self.trajectory[i-1].x >= x > self.trajectory[i].x) and\
                (self.trajectory[i-1].y <= y < self.trajectory[i].y or self.trajectory[i-1].y >= y > self.trajectory[i].y):
                print("correct intersection")
                return Pose(x,y)
            
            i += 1

    def resample_trajectory23(self,path_res):

        projection_yaw = np.deg2rad((np.rad2deg(self.ego_pose.yaw)+360+90)%360)

        projection_x1 = self.ego_pose.x
        projection_y1 = self.ego_pose.y 
        projection_x2 = self.ego_pose.x + (np.cos(projection_yaw))
        projection_y2 = self.ego_pose.y + (np.sin(projection_yaw))

        length_on_trajectory = None

        i = 1

        while i < len(self.trajectory):

            intersect_x,intersect_y = self.find_intersection(self.trajectory[i-1].x,self.trajectory[i].x,self.trajectory[i-1].y,self.trajectory[i].y,\
                                            projection_x1,projection_x2,projection_y1,projection_y2)

            if self.trajectory[i-1].x > intersect_x > self.trajectory[i].x or self.trajectory[i-1].x < intersect_x < self.trajectory[i].x and\
                self.trajectory[i-1].y > intersect_y > self.trajectory[i].y or self.trajectory[i-1].y < intersect_y < self.trajectory[i].y:
                
                dx = abs(intersect_x-self.trajectory[i-1].x)
                dy = abs(intersect_y-self.trajectory[i-1].y)
                hyp = math.hypot(dx, dy)

                length_on_trajectory = self.trajectory[i-1].length + hyp

                break
            
            i += 1

        trajectory23 = []
        j = 1
        trajectory23_cnt = 0

        while j < len(self.trajectory) and trajectory23_cnt < 23:
            
            if self.trajectory[j-1].length < length_on_trajectory < self.trajectory[j].length:

                new_traj_point = TrajectoryPoint(length_on_trajectory)

                new_traj_point.x = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j-1].x,\
                                                            self.trajectory[j].length,self.trajectory[j].x,length_on_trajectory)

                new_traj_point.y = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j-1].y,\
                                                            self.trajectory[j].length,self.trajectory[j].y,length_on_trajectory)
                
                new_traj_point.yaw = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j-1].yaw,\
                                                            self.trajectory[j].length,self.trajectory[j].yaw,length_on_trajectory)

                new_traj_point.curvature = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j-1].curvature,\
                                                            self.trajectory[j].length,self.trajectory[j].curvature,length_on_trajectory)

                new_traj_point.velocity = self.calc_lin_interpol(self.trajectory[j-1].length,self.trajectory[j-1].velocity,\
                                                            self.trajectory[j].length,self.trajectory[j].velocity,length_on_trajectory)

                trajectory23.append(new_traj_point)

                length_on_trajectory += path_res
                trajectory23_cnt += 1
                j = 0
                
            j += 1
            
    @staticmethod
    def calc_lin_interpol(x1,x2,y1,y2,x3):
        m = y2-y1/x2-x1   
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
    def distance_angle_PoseA_to_PoseB(PoseA,PoseB):
        dx = PoseA.x - PoseB.x
        dy = PoseA.y - PoseB.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


class PlannerMachine(StateMachine):

    planner = G2ClothoidPlanner()

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
        print('State: goalreached_check')

        d, theta = self.planner.distance_angle_PoseA_to_PoseB(self.planner.ego_pose,self.planner.goal_pose)

        if d <= 0.1 and theta <= 0.1:
            print('!goal reached!')
            self.goalreached_check_success()
        
        else:
            print('!goal not reached!')
            self.goalreached_check_fail()

    def on_enter_goalreached(self):
        print('State: goalreached')

        self.goalreached_reset()
    
    def on_enter_gen_trajectory(self):
        print('State: gen_trajectory')

        self.planner.generate_trajectory()

        self.trajectory_generated_success()

    def on_enter_bilevel_check(self):
        print('State: bilevel_check')

        if self.planner.trajectory:
        
            dis_goal_trajgoal, theta_goal_trajgoal = self.planner.distance_angle_PoseA_to_PoseB(self.planner.trajectory[-1],self.planner.goal_pose)

            point_on_trajectory = self.planner.give_projection_point_on_trajectory()
            dis_ego_traj, theta_ego_traj = self.planner.distance_angle_PoseA_to_PoseB(point_on_trajectory,self.planner.ego_pose)

            #---TransitionCondition---#
            if dis_goal_trajgoal <= 1:# and theta_goal_trajgoal <= np.deg2rad(10):
                if dis_ego_traj <= 3:# and theta_ego_traj <= np.deg2rad(10):

                    self.bilevel_check_success()
                else:
                    print("ego Pose not on trajectory")
                    #self.bilevel_check_fail()
            else:   
                print("goal not on trajectory")
                #self.bilevel_check_fail()
        else:
            print("not found trajectory")
            self.bilevel_check_fail()

    def on_enter_feasibility_check(self):
        print('State: feasibility_check')

        curvature_feasible = True

        for trajectory_point in self.planner.trajectory:
            if trajectory_point.curvature > 0.5:
                curvature_feasible = False

        #---TransitionCondition---#
        if self.planner.trajectory[-1].length > 2 and curvature_feasible:
            self.feasibility_check_success()
        else:
            self.feasibility_check_fail()

    def on_enter_send_trajectory(self):
        print('State: send_trajectory')

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

class G1ClothoidPlanner:

    trajectory = []
    trajectory23 = []
    
    def __init__(self):
        pass

    def generate_trajectory(self, ego_pose, goal_pose):

        g2clothoid_list = pyclothoids.Clothoid.G1Hermite(ego_pose.x, ego_pose.y, ego_pose.yaw, goal_pose.x, goal_pose.y, goal_pose.yaw)
        g2clothoid_trajectory = self.sample_trajectory_HP(g2clothoid_list)

        self.trajectory = g2clothoid_trajectory
        self.trajectory23 = g2clothoid_trajectory[0:22]
    
        return self.trajectory

    @staticmethod
    def sample_trajectory(g2clothoid_list):

        trajectory_point_list = []
  
        for clothoid in g2clothoid_list:
            trajectory_point_list_temp = [TrajectoryPoint(clothoid.length * m/(round(clothoid.length/0.1))) for m in range(0,round(clothoid.length/0.1))]

            for trajectory_point in trajectory_point_list_temp:
                trajectory_point.x = clothoid.X(trajectory_point.length)
                trajectory_point.y = clothoid.Y(trajectory_point.length)
                trajectory_point.yaw = clothoid.Theta(trajectory_point.length)
                trajectory_point.curvature = clothoid.KappaStart + (clothoid.dk*trajectory_point.length)
                trajectory_point.velocity = 0.5
        
            trajectory_point_list += trajectory_point_list_temp

        return trajectory_point_list

    @staticmethod
    def sample_trajectory_HP(g1clothoid):

        total_length = g1clothoid.length
        npt_tar = round(total_length/0.1)
        samp_int = total_length/npt_tar

        trajectory_point_list = []
        samp_point = 0
        
        while samp_point < g1clothoid.length:

            trajectory_point_list.append(TrajectoryPoint(samp_point,
                                                        g1clothoid.X(samp_point),
                                                        g1clothoid.Y(samp_point),
                                                        g1clothoid.Theta(samp_point),
                                                        g1clothoid.KappaStart + (g1clothoid.dk*samp_point)))

            samp_point += samp_int

        trajectory_point_list.append(TrajectoryPoint(total_length,
                                                    g1clothoid.X(g1clothoid.length),
                                                    g1clothoid.Y(g1clothoid.length),
                                                    g1clothoid.Theta(g1clothoid.length),
                                                    g1clothoid.KappaStart + (g1clothoid.dk*g1clothoid.length)))

        return trajectory_point_list

        
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
        self.driven_trajectory = []
        self.ego_on_trajectorylength = 0
        self.drive_step = 0.5

        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None

    def ego_drive_step(self):

        self.ego_on_trajectorylength += self.drive_step

        i = 1

        while i < len(self.plannermachine.planner.trajectory):

            if self.plannermachine.planner.trajectory[i-1].length < self.ego_on_trajectorylength < self.plannermachine.planner.trajectory[i].length:
                
                """
                x = self.plannermachine.planner.calc_lin_interpol(self.plannermachine.planner.trajectory[i-1].length,self.plannermachine.planner.trajectory[i-1].x,\
                                                self.plannermachine.planner.trajectory[i].length,self.plannermachine.planner.trajectory[i].x,self.ego_on_trajectorylength)

                y = self.plannermachine.planner.calc_lin_interpol(self.plannermachine.planner.trajectory[i-1].length,self.plannermachine.planner.trajectory[i-1].y,\
                                                self.plannermachine.planner.trajectory[i].length,self.plannermachine.planner.trajectory[i].y,self.ego_on_trajectorylength)
                
                yaw = self.plannermachine.planner.calc_lin_interpol(self.plannermachine.planner.trajectory[i-1].length,self.plannermachine.planner.trajectory[i-1].yaw,\
                                                self.plannermachine.planner.trajectory[i].length,self.plannermachine.planner.trajectory[i].yaw,self.ego_on_trajectorylength)
                """

                self.ego.x -= x + np.random.normal(0,0.1)
                self.ego.y -= y + np.random.normal(0,0.1)
                self.ego.yaw = yaw


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
            
            self.ego_drive_step()

            self.visualization()

            time.sleep(1)
        

        
    def visualization(self, rnd = None):
        plt.figure(2)
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

        #self.LinePlot()

    def LinePlot(self):

        plt.figure(1)
        
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
        
        plt.figure(2)

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
    simulation = Simulation(init_pose=[10,17.5, np.deg2rad(-150),0],
                            goal_pose=[3,5, np.deg2rad(-90),0],
                            play_area=[0,20,0,20])

    simulation.simulate()

    #Test

    print("finished")
    plt.show()

if __name__ == '__main__':
    main()