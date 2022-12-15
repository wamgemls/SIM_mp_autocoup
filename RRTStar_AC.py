import math
import random

import matplotlib.pyplot as plt
import numpy as np

import pyclothoids as clothoid

show_animation = True


class RRT:
    
    class Node:

        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta
            self.path_x = []
            self.path_y = []
            self.cost = 0.0
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])


    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=2,
                 path_resolution=0.01,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):

        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.end.cost = float("inf")
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.final_course =[]
        self.robot_radius = robot_radius

    def planning(self, animation=True):

        self.node_list = [self.start]

        for i in range(self.max_iter):


            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            
            if animation and i % 50 == 0:
                self.draw_graph(rnd_node)
            
            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                
                near_nodes = self.nodes_in_rad(new_node)
                #new_node = self.choose_parent(near_nodes,nearest_node,new_node)
                new_node = self.choose_parent2(near_nodes, new_node)
                self.node_list.append(new_node)
                self.rewire2(near_nodes, new_node)

            
            
            if self.calc_dist_to_goal(new_node.x, new_node.y) <= self.expand_dis:

                tmp_end = self.steer(new_node, self.end)
                
                if tmp_end.cost < self.end.cost:

                    if self.check_collision(self.end, self.obstacle_list, self.robot_radius):

                        self.end = tmp_end
                        self.final_course = self.generate_final_course()
            
           
                    
                    #return final_course
            

        #return None  # cannot find path

    def choose_parent2(self, near_nodes, new_node):

        if len(near_nodes) == 0:
            return new_node

        costlist = []
        for node in near_nodes:
            d, theta = self.calc_distance_and_angle(node, new_node)

            tmp_node = self.steer(node,new_node)

            if self.check_collision(tmp_node,self.obstacle_list,self.robot_radius):
                costlist.append(node.cost + d)
            else:
                costlist.append(float("inf"))

        mincost = min(costlist)
        minind = near_nodes[costlist.index(mincost)]

        if mincost == float("inf"):
            print("mincost is inf")
            return new_node

        new_node = self.steer(minind,new_node)
        new_node.parent = minind

        return new_node

    def rewire2(self, near_nodes,new_node):

        for node in near_nodes:

            d, theta = self.calc_distance_and_angle(new_node, node)

            if new_node.cost + d < node.cost:

                tmp_node = self.steer(new_node, node)

                if self.check_collision(tmp_node,self.obstacle_list,self.robot_radius):
                    node = tmp_node
                    print("Rewiring")
    
    def nodes_in_rad(self,new_node):
        
        nnode = len(self.node_list)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))

        near_list=[]

        for node in self.node_list:
            
            d, theta = self.calc_distance_and_angle(node, new_node)

            if d <= r:
                near_list.append(node)

        return near_list

    def steer2goal(self, from_node, to_node, extend_length=float("inf")):
        
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        new_node = self.Node(to_node.x,to_node.y, to_node.theta)


        clothoid_listG1 = clothoid.Clothoid.G1Hermite(from_node.x, from_node.y ,from_node.theta, new_node.x ,new_node.y ,new_node.theta)
        new_node.path_x, new_node.path_y = clothoid_listG1.SampleXY(50)

        new_node.parent = from_node
        new_node.cost = from_node.cost + clothoid_listG1.length


        return new_node

    def steer2(self, from_node, to_node, extend_length=float("inf")):
        
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        new_node = self.Node(from_node.x,from_node.y, theta)

        if extend_length > d:
            extend_length = d

        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)

        clothoid_listG1 = clothoid.Clothoid.G1Hermite(from_node.x, from_node.y ,from_node.theta, new_node.x ,new_node.y ,new_node.theta)
        new_node.path_x, new_node.path_y = clothoid_listG1.SampleXY(50)

        new_node.parent = from_node

        new_node.cost = from_node.cost + clothoid_listG1.length
        
        return new_node

    def steer(self, from_node, to_node, expand_dis=float("inf")):

        new_node = self.Node(from_node.x, from_node.y, None)
        d, theta = self.calc_distance_and_angle(from_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        # if no expand_dis is given -> expand_dis == dis between nodes

        if d < expand_dis:
            expand_dis = d

        new_node.cost = from_node.cost + expand_dis

        n_expand = math.floor(expand_dis / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        #final connection (path resolution)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.cost += d

        new_node.parent = from_node

        return new_node

    def generate_final_course(self):
        path = [self.end]
        node = self.end
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(node)

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                None)
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y,None)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')

    
        for node in self.node_list:
            plt.plot(node.x,node.y, "xb")
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for node in self.final_course:
            plt.plot(node.path_x, node.path_y, "-r")
                

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

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
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if len(d_list) == 0:
                return True

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision


        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def main():
    print("start " + __file__)

    
    obstacleList = [[12,4,3],[6,2,2],[3,10,2],[3,13,2]
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0, np.deg2rad(45)],
        goal=[12, 10, np.deg2rad(0)],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.0
        )
    
    rrt.planning(animation=show_animation)

    """
    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
    """
    print("finished")

    plt.show()

if __name__ == '__main__':
    main()