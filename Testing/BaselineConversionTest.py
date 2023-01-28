import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import pyclothoids as clothoid

import time
import math

class Node:

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.path_x = []
        self.path_y = []
        self.cost = 0.0
        self.parent = None

max_curvature = 0.5
max_curvature_rate = max_curvature/1

clot = clothoid.Clothoid.StandardParams(0, 0,np.deg2rad(45),0, 0.5, 3)
path_x, path_y = clot.SampleXY(50)

def node_cost(from_node, to_node):

  _,bl_angle = calc_distance_and_angle(from_node, to_node)
  broken_node = Node(None,None,None)
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
    
    if (normal_to_node_theta > ((normal_from_node_theta - 180)+360) % 360):
      
      inter_node = find_intersection(from_node, to_node)
      new_node = gen_triangle(from_node, to_node, inter_node)
      new_node.cost = new_node.cost + from_node.cost
      print("fine case 1a - Cost: ", new_node.cost)

      return new_node

    else:
      print("broken case 1a")

      return broken_node


  elif (180 < normal_from_node_theta < 360 and 0 < normal_to_node_theta < 180):
  
    if (normal_to_node_theta < ((normal_from_node_theta + 180)+360) % 360):

      inter_node = find_intersection(from_node, to_node)
      new_node = gen_triangle(from_node, to_node, inter_node)
      new_node.cost = new_node.cost + from_node.cost
      print("fine case 1b - Cost: ", new_node.cost)

      #easyg1
      new_node2 = g1_steer(from_node,to_node)
      plt.plot(new_node2.path_x,new_node2.path_y, "b")
      print("g1_alternative - Cost: ", new_node2.cost)

      #easyg2
      new_node3 = g2_steer(from_node,to_node)
      plt.plot(new_node3.path_x,new_node3.path_y, "y")
      print("g2_alternativr - Cost: ", new_node3.cost)



      return new_node

    else:
      print("broken case 1b")

      return broken_node

  elif (0 < normal_from_node_theta <= 90 and 0 < normal_to_node_theta < 90) or\
      (270 <= normal_from_node_theta < 360 and 270 < normal_to_node_theta < 360):

    mid_node_theta = -(normal_from_node_theta+normal_to_node_theta)/2
    mid_node = Node((from_node.x+to_node.x)/2,(from_node.y+to_node.y)/2,np.deg2rad(mid_node_theta+bl_degree))

    plot_arrow(mid_node.x,mid_node.y,mid_node.theta)

    inter_node1 = find_intersection(from_node, mid_node)
    tempNode1 = gen_triangle(from_node, mid_node, inter_node1)

    inter_node2 = find_intersection(mid_node, to_node)
    tempNode2 = gen_triangle(mid_node, to_node, inter_node2)

    tempNode1.path_x += tempNode2.path_x
    tempNode1.path_y += tempNode2.path_y

    tempNode1.cost += tempNode2.cost + from_node.cost

    tempNode1.x = tempNode2.x
    tempNode1.y = tempNode2.y

    print("case 2 - Cost: ", tempNode1.cost)

    return tempNode1
    
  else: 

    print("is mist")

    return broken_node

def gen_triangle(from_node, to_node, inter_node):

  dis_fn,_ = calc_distance_and_angle(from_node,inter_node)
  dis_tn,_ = calc_distance_and_angle(to_node,inter_node)

  #generating isosceles triangle on from_node oder to_node side

  if dis_fn < dis_tn:
    tri_node = find_point3(inter_node, to_node, dis_fn)
    plt.plot([tri_node.x], [tri_node.y], "xr")
    print("case A")

    tempNode1 = g1_steer(from_node, tri_node)
    tempNode2 = line_steer(tri_node,to_node)

    tempNode1.path_x += tempNode2.path_x
    tempNode1.path_y += tempNode2.path_y

    tempNode1.cost = tempNode1.cost + tempNode2.cost

    tempNode1.x = tempNode2.x
    tempNode1.y = tempNode2.y

    plt.plot(tempNode1.path_x, tempNode1.path_y, "-g")

    return tempNode1

  elif dis_tn < dis_fn:
    tri_node = find_point3(inter_node, from_node, dis_tn)
    plt.plot([tri_node.x], [tri_node.y], "xr")

    print("case B")
    tempNode1 = line_steer(from_node, tri_node)
    tempNode2 = g1_steer(tri_node,to_node)

    tempNode1.path_x += tempNode2.path_x
    tempNode1.path_y += tempNode2.path_y

    tempNode1.cost = tempNode1.cost + tempNode2.cost

    tempNode1.x = tempNode2.x
    tempNode1.y = tempNode2.y

    plt.plot(tempNode1.path_x, tempNode1.path_y, "-g")
    
    return tempNode1

  else:
    #intersection point is in exact mid
    print("intersection point is in exact mid")

    tempNode = g1_steer(from_node,to_node)
    plt.plot(tempNode.path_x, tempNode.path_y, "-g")

    return tempNode


def find_point3(inter_node, pointing_node, dis):

    tempNode1 = Node(inter_node.x - (np.cos(pointing_node.theta)*dis),inter_node.y - (np.sin(pointing_node.theta)*dis), pointing_node.theta)
    tempNode2 = Node(inter_node.x + (np.cos(pointing_node.theta)*dis),inter_node.y - (np.sin(pointing_node.theta)*dis), pointing_node.theta)
    tempNode3 = Node(inter_node.x - (np.cos(pointing_node.theta)*dis),inter_node.y + (np.sin(pointing_node.theta)*dis), pointing_node.theta)
    tempNode4 = Node(inter_node.x + (np.cos(pointing_node.theta)*dis),inter_node.y + (np.sin(pointing_node.theta)*dis), pointing_node.theta)
    
    temp_nodelist = [tempNode1,tempNode2,tempNode3,tempNode4]
    dis_list = []

    for temp_node in temp_nodelist:

      dis,_ = calc_distance_and_angle(temp_node, pointing_node)
      dis_list.append(dis)

    mindis = min(dis_list)
    tri_node = temp_nodelist[dis_list.index(mindis)]

    plot_arrow(tri_node.x,tri_node.y,tri_node.theta)

    return tri_node

def find_intersection(from_node, to_node):

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

    inter_node = Node(x,y,None)

    plt.plot([inter_node.x], [inter_node.y], "x")

    return inter_node

def calc_distance_and_angle(from_node, to_node):
  dx = to_node.x - from_node.x
  dy = to_node.y - from_node.y
  d = math.hypot(dy,dx)
  theta = math.atan2(dy,dx)
    
  return d, theta

def plot_arrow(x, y, yaw, length=0.15, width=0.15, fc="r", ec="k"):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)

def g1_steer(from_node, to_node):
  
  new_node = Node(to_node.x,to_node.y,to_node.theta)

  g1_clothoid = clothoid.Clothoid.G1Hermite(from_node.x, from_node.y ,from_node.theta, to_node.x ,to_node.y ,to_node.theta)
  new_node.path_x, new_node.path_y = g1_clothoid.SampleXY(50)
  new_node.parent = from_node

  if -max_curvature < g1_clothoid.KappaStart < max_curvature and -max_curvature < g1_clothoid.KappaEnd < max_curvature:
    new_node.cost = g1_clothoid.length
  else:
    new_node.cost = math.inf
  
  return new_node




def g2_steer(from_node, to_node):
  
  new_node = Node(to_node.x,to_node.y,to_node.theta)

  g2_clothoid_list = clothoid.SolveG2(from_node.x, from_node.y, from_node.theta, 0, to_node.x, to_node.y, to_node.theta, 0)
  
  for clot in g2_clothoid_list:

    pathx, pathy = clot.SampleXY(50)

    new_node.path_x += pathx
    new_node.path_y += pathy
    
    if -max_curvature < clot.KappaStart < max_curvature and -max_curvature < clot.KappaEnd < max_curvature:
      
      new_node.cost += clot.length
    
    else:
      new_node.cost = math.inf
  

  new_node.parent = from_node

  return new_node

def line_steer(from_node, to_node, expand_dis=float("inf")):

  d, theta = calc_distance_and_angle(from_node, to_node)
  new_node = Node(from_node.x, from_node.y, theta)
        
  new_node.path_x = [new_node.x]
  new_node.path_y = [new_node.y]

  # if no expand_dis is given -> expand_dis == dis between nodes

  if d < expand_dis:
    expand_dis = d

  new_node.cost = expand_dis


  n_expand = math.floor(expand_dis/0.01)

  for _ in range(n_expand):
    new_node.x += 0.01 * math.cos(theta)
    new_node.y += 0.01 * math.sin(theta)
    new_node.path_x.append(new_node.x)
    new_node.path_y.append(new_node.y)

  #final connection (path resolution)

  d, _ = calc_distance_and_angle(new_node, to_node)
  if d <= 0.01:
    new_node.path_x.append(to_node.x)
    new_node.path_y.append(to_node.y)
    new_node.x = to_node.x
    new_node.y = to_node.y
    new_node.cost += d

  new_node.parent = from_node

  return new_node

p1 = Node(5,5,np.deg2rad(-2))

p2 = Node(10,5,np.deg2rad(10))

plot_arrow(p1.x,p1.y,p1.theta)
plot_arrow(p2.x,p2.y,p2.theta)

node_cost(p1,p2)

plt.plot([p1.x, p2.x], [p1.y, p2.y], "-r")

plt.axis([-2, 10, -6, 6])

plt.axis("equal")

plt.grid(True)


plt.show()




