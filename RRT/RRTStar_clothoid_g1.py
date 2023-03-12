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
                 expand_dis=3,
                 path_resolution=0.01,
                 goal_sample_rate=15,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 max_curvature=0.5
                 ):

        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        #self.end.cost = float("inf")
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
        self.node_list2 = []
        self.final_course = []
        self.robot_radius = robot_radius
        self.max_curvature = max_curvature
        self.max_curvature_rate = max_curvature/expand_dis

    def planning(self, animation=True):

        self.node_list = [self.start]
        self.node_list2 = [self.end]

        for i in range(self.max_iter):
            
            print(i)

            self.tree_building(self.start,self.node_list2,animation)

            self.tree_building(self.end,self.node_list,animation)      
        
            #def nodecost(self,n)

    def tree_building(self, goal, node_tree, animation):

        rnd_node = self.get_random_node(goal)
        distance_sorted_list = self.get_distance_sorted_list(node_tree,rnd_node)
        nearest_node_iter = iter(distance_sorted_list)
        tick_planned = False
        
        #nearest_ind = self.get_nearest_node_index(node_tree, rnd_node)
        #nearest_node = node_tree[nearest_ind]
        #new_node = self.clothoid_steer(nearest_node, rnd_node, self.expand_dis)
        length = len(distance_sorted_list)
        i=0


        while tick_planned is False and i < length:

            new_node = self.clothoid_steer(next(nearest_node_iter), rnd_node, self.expand_dis)

            #print("nearest nr: ", i)

            if self.check_if_outside_play_area(new_node, self.play_area) \
                and self.check_collision(new_node,self.obstacle_list,self.robot_radius):

                node_already_existent = False
                for node in node_tree:

                    dis,_ = self.calc_distance_and_angle(node, new_node)

                    if dis < 0.2:
                        node_already_existent = True
                        #print("existiert bereits")

                if not node_already_existent:
        
                    near_nodes = self.nodes_in_rad(new_node, node_tree)

                    for near_node in near_nodes:

                        new_temp_node = self.g1_steer(near_node, new_node)
                        
                        if new_temp_node.cost < new_node.cost:

                            if self.check_collision(new_temp_node,self.obstacle_list,self.robot_radius):
                                new_node = new_temp_node
                                print("Updating Newnode Parent")
                
                    node_tree.append(new_node)

                    for near_node in near_nodes:
                        cnear = near_node.cost
                        new_temp_node = self.g1_steer(new_node, near_node)

                        if new_temp_node.cost < cnear:

                            if self.check_collision(new_temp_node,self.obstacle_list,self.robot_radius):
                                node_tree.remove(near_node)
                                node_tree.append(new_temp_node)
                                print("Rewiring")
                    tick_planned = True

            i += 1
        
        if animation:# and i % 50 == 0:
            self.draw_graph(rnd_node)

    
    def nodes_in_rad(self,new_node, node_list):
        nnode = len(node_list)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        near_list = []

        for node in node_list:
            if node != new_node:
                d, theta = self.calc_distance_and_angle(node, new_node)
                if d <= r:
                    near_list.append(node)

        return near_list

    def node_cost(self,from_node, to_node):

        _,bl_angle = self.calc_distance_and_angle(from_node, to_node)
        broken_node = self.Node(None,None,None)
        broken_node.cost = math.inf

        #rad2deg (base 360)

        from_node_theta = (np.rad2deg(from_node.theta)+360)%360
        to_node_theta = (np.rad2deg(to_node.theta)+360)%360
        bl_degree = (np.rad2deg(bl_angle)+360)%360

        #offsetting angles

        normal_from_node_theta = ((from_node_theta - bl_degree)+360)%360
        normal_to_node_theta = ((to_node_theta - bl_degree)+360)%360

        #precheck whether theta are aligned closely
        #decide over or under baseline

        if (0 < normal_from_node_theta < 180 and 180 < normal_to_node_theta < 360):
            
            if (normal_to_node_theta > ((normal_from_node_theta - 180) +360) % 360):
            
                inter_node = self.find_intersection(from_node, to_node)
                new_node = self.gen_triangle(from_node, to_node, inter_node)
                new_node.cost = new_node.cost + from_node.cost
                new_node.parent = from_node
                #print("fine case 1a - Cost: ", new_node.cost)
                return new_node

            else:
                #print("broken case 1a")
                return broken_node


        elif (180 < normal_from_node_theta < 360 and 0 < normal_to_node_theta < 180):
        
            if (normal_to_node_theta < ((normal_from_node_theta + 180)+360) % 360):

                inter_node = self.find_intersection(from_node, to_node)
                new_node = self.gen_triangle(from_node, to_node, inter_node)
                new_node.cost = new_node.cost + from_node.cost
                new_node.parent = from_node
                #print("fine case 1b - Cost: ", new_node.cost)
                return new_node

            else:
                #print("broken case 1b")
                return broken_node

        elif (0 < normal_from_node_theta <= 90 and 0 < normal_to_node_theta < 90) or\
                (270 <= normal_from_node_theta < 360 and 270 < normal_to_node_theta < 360):

            mid_node_theta = -(normal_from_node_theta+normal_to_node_theta)/2
            mid_node = self.Node((from_node.x+to_node.x)/2,(from_node.y+to_node.y)/2,np.deg2rad(mid_node_theta+bl_degree))

            #self.plot_arrow(mid_node.x,mid_node.y,mid_node.theta)

            inter_node1 = self.find_intersection(from_node, mid_node)
            tempNode1 = self.gen_triangle(from_node, mid_node, inter_node1)

            inter_node2 = self.find_intersection(mid_node, to_node)
            tempNode2 = self.gen_triangle(mid_node, to_node, inter_node2)

            new_node = self.Node(tempNode2.x,tempNode2.y, tempNode2.theta)
            new_node.path_x = tempNode1.path_x + tempNode2.path_x
            new_node.path_y = tempNode1.path_y + tempNode2.path_y
            new_node.cost = from_node.cost + tempNode1.cost + tempNode2.cost
            new_node.parent = from_node

            #print("case 2 - Cost: ", tempNode1.cost)

            return new_node
            
        else: 
            return broken_node
            
    def gen_triangle(self, from_node, to_node, inter_node):

        dis_fn,_ = self.calc_distance_and_angle(from_node,inter_node)
        dis_tn,_ = self.calc_distance_and_angle(to_node,inter_node)

        #generating isosceles triangle on from_node oder to_node side

        if dis_fn < dis_tn:
            tri_node = self.find_point3(inter_node, to_node, dis_fn)
            #plt.plot([tri_node.x], [tri_node.y], "xr")

            tempNode1 = self.g1_steer(from_node, tri_node)
            tempNode2 = self.line_steer(tri_node,to_node)

            new_node = self.Node(tempNode2.x,tempNode2.y, tempNode2.theta)
            new_node.path_x = tempNode1.path_x + tempNode2.path_x
            new_node.path_y = tempNode1.path_y + tempNode2.path_y
            new_node.cost = tempNode1.cost + tempNode2.cost
            new_node.parent = from_node

            #plt.plot(tempNode1.path_x, tempNode1.path_y, "-g")

            return new_node

        elif dis_tn < dis_fn:
            tri_node = self.find_point3(inter_node, from_node, dis_tn)
            #plt.plot([tri_node.x], [tri_node.y], "xr")

            tempNode1 = self.line_steer(from_node, tri_node)
            tempNode2 = self.g1_steer(tri_node,to_node)

            new_node = self.Node(tempNode2.x,tempNode2.y, tempNode2.theta)
            new_node.path_x = tempNode1.path_x + tempNode2.path_x
            new_node.path_y = tempNode1.path_y + tempNode2.path_y
            new_node.cost = tempNode1.cost + tempNode2.cost
            new_node.parent = from_node

            #plt.plot(tempNode1.path_x, tempNode1.path_y, "-g")
            
            return new_node
        else:
            #intersection point is in exact mid
            print("intersection point is in exact mid")
            new_node = self.g1_steer(from_node,to_node)
            new_node.parent = from_node
            
            #plt.plot(tempNode.path_x, tempNode.path_y, "-g")

            return new_node


    def find_point3(self,inter_node, pointing_node, dis):

        tempNode1 = self.Node(inter_node.x - (np.cos(pointing_node.theta)*dis),inter_node.y - (np.sin(pointing_node.theta)*dis), pointing_node.theta)
        tempNode2 = self.Node(inter_node.x + (np.cos(pointing_node.theta)*dis),inter_node.y - (np.sin(pointing_node.theta)*dis), pointing_node.theta)
        tempNode3 = self.Node(inter_node.x - (np.cos(pointing_node.theta)*dis),inter_node.y + (np.sin(pointing_node.theta)*dis), pointing_node.theta)
        tempNode4 = self.Node(inter_node.x + (np.cos(pointing_node.theta)*dis),inter_node.y + (np.sin(pointing_node.theta)*dis), pointing_node.theta)
        
        temp_nodelist = [tempNode1,tempNode2,tempNode3,tempNode4]
        dis_list = []

        for temp_node in temp_nodelist:

            dis,_ = self.calc_distance_and_angle(temp_node, pointing_node)
            dis_list.append(dis)

            mindis = min(dis_list)
            tri_node = temp_nodelist[dis_list.index(mindis)]

            #self.plot_arrow(tri_node.x,tri_node.y,tri_node.theta)

        return tri_node

    def find_intersection(self,from_node, to_node):

    #generate second point for intersection finder

        from_nodex2 = from_node.x + (np.cos(from_node.theta))
        from_nodey2 = from_node.y + (np.sin(from_node.theta))
        to_nodex2 = to_node.x + (np.cos(to_node.theta))
        to_nodey2 = to_node.y + (np.sin(to_node.theta))

        xdiff = (from_node.x - from_nodex2, to_node.x - to_nodex2)
        ydiff = (from_node.y - from_nodey2, to_node.y - to_nodey2)

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            raise Exception('lines do not intersect')

        d = (det((from_node.x,from_node.y),(from_nodex2,from_nodey2)), \
            det((to_node.x, to_node.y),(to_nodex2,to_nodey2)))

        x = det(d, xdiff) / div
        y = det(d, ydiff) / div

        inter_node = self.Node(x,y,None)

        #plt.plot([inter_node.x], [inter_node.y], "x")

        return inter_node


    def steer2goal(self, from_node, to_node):
        new_node = self.Node(to_node.x, to_node.y, to_node.theta)
        clothoid_listG1 = clothoid.Clothoid.G1Hermite(from_node.x, from_node.y ,from_node.theta, to_node.x ,to_node.y ,to_node.theta)
        new_node.path_x, new_node.path_y = clothoid_listG1.SampleXY(50)
        new_node.parent = from_node
        new_node.cost = from_node.cost + clothoid_listG1.length

        return new_node

    def clothoid_steer(self, from_node,rnd_node, arc_len):
        dmin = float("inf")
        kappa_start = None
        kappa_rate = None
        new_node = None

        #curvature constraints
        for kd in np.arange(-self.max_curvature,self.max_curvature,0.01):
            clot = clothoid.Clothoid.StandardParams(from_node.x, from_node.y ,from_node.theta,kd, 0, arc_len)

            dx = clot.XEnd - rnd_node.x
            dy = clot.YEnd - rnd_node.y
            d = math.hypot(dx, dy)
            if  d < dmin:
                dmin = d
                new_node = self.Node(clot.XEnd,clot.YEnd,clot.ThetaEnd)
                new_node.path_x, new_node.path_y = clot.SampleXY(50)
                kappa_start = clot.KappaStart
                kappa_rate = clot.dk
       
        new_node.cost = from_node.cost + arc_len
        new_node.parent = from_node

        #print("kappa start: ", kappa_start, " kapparate: ", kappa_rate)

        return new_node

    def clothoid_steer2(self, from_node,rnd_node, arc_len):
        dmin = float("inf")
        kopt = None
        new_node = None

        #curvature constraints
        for kd in np.arange(-self.max_curvature_rate,self.max_curvature_rate,0.01):
            clot = clothoid.Clothoid.StandardParams(from_node.x, from_node.y ,from_node.theta,0, kd, arc_len)

            dx = clot.XEnd - rnd_node.x
            dy = clot.YEnd - rnd_node.y
            d = math.hypot(dx, dy)
            if  d < dmin:
                dmin = d
                new_node = self.Node(clot.XEnd,clot.YEnd,clot.ThetaEnd)
                new_node.path_x, new_node.path_y = clot.SampleXY(50)
       
        new_node.cost = from_node.cost + arc_len
        new_node.parent = from_node

        return new_node


    def g1_steer(self,from_node, to_node):
        #generating G1fitting between two nodes
        new_node = self.Node(to_node.x,to_node.y,to_node.theta)

        g1_clothoid = clothoid.Clothoid.G1Hermite(from_node.x, from_node.y ,from_node.theta, to_node.x ,to_node.y ,to_node.theta)
        new_node.path_x, new_node.path_y = g1_clothoid.SampleXY(50)

        new_node.parent = from_node

        if -self.max_curvature < g1_clothoid.KappaStart < self.max_curvature and -self.max_curvature < g1_clothoid.KappaEnd < self.max_curvature:
            new_node.cost = g1_clothoid.length
        else:
            new_node.cost = math.inf

        return new_node

    def line_steer(self, from_node, to_node, expand_dis=float("inf")):
        #generating LineConnection between two nodes
        d, theta = self.calc_distance_and_angle(from_node, to_node)
        new_node = self.Node(from_node.x, from_node.y, theta)
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        # if no expand_dis is given -> expand_dis == dis between nodes
        if d < expand_dis:
            expand_dis = d

        new_node.cost = expand_dis
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

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self, heuristic):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                None)
        else:  # goal point sampling
            rnd = self.Node(heuristic.x, heuristic.y,None)

        return rnd

    def generate_final_course(self):
        path = []
        node = self.end
        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(node)

        i = 0
        for node in path:
            print(i," ",node)
            i+=1

        return path

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
            #plt.plot(node.x,node.y, "xb")
            self.plot_arrow(node.x,node.y,node.theta)
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-b")

        for node in self.node_list2:
            #plt.plot(node.x,node.y, "xb")
            self.plot_arrow(node.x,node.y,node.theta)
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-r")

        for node in self.final_course:
            plt.plot(node.x,node.y, "xy")
            plt.plot(node.path_x, node.path_y, "-r")
                
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size,"y")

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

        #radius = 1/self.max_curvature
        #self.plot_circle(self.start.x,self.start.y+radius, radius, "-r")
        
        plt.axis([-2, 22, -2, 22])
        plt.grid(True)
        #plt.axis("equal")
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_distance_sorted_list(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        comb = sorted(zip(node_list,dlist),key= lambda x:x[1])
        sorted_list = [x[0] for x in comb]
        
        return sorted_list

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

    @staticmethod
    def plot_arrow(x, y, yaw, length=0.15, width=0.15, fc="r", ec="k"):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    print("start " + __file__)
    
    obstacleList = [[3,13,2],[11,13,2],[5,5,2],[13,5,2]
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[5, 17, np.deg2rad(0)],
        goal=[9,3, np.deg2rad(90)],
        rand_area=[-5,25],
        expand_dis=2.5,
        obstacle_list=obstacleList,
        play_area=[0,20,0,20],
        robot_radius=0.1,
        max_curvature=0.25
        )
    
    rrt.planning(animation=show_animation)

    print("finished")

    plt.show()

if __name__ == '__main__':
    main()