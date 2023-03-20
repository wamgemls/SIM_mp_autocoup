import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patches as mpatches
from matplotlib.legend_handler import HandlerPatch
from matplotlib.legend import Legend
import csv
from matplotlib.offsetbox import AnchoredText
import matplotlib.gridspec as gridspec

import numpy as np
import os

class AnyObject:
    pass


class AnyObjectHandler:
    def legend_artist(self, legend, orig_handle, fontsize, handlebox):
        x0, y0 = handlebox.xdescent, handlebox.ydescent
        width, height = handlebox.width, handlebox.height
        patch = mpatches.Rectangle([x0, y0], width, height, facecolor='red',
                                   edgecolor='black', hatch='xx', lw=3,
                                   transform=handlebox.get_transform())
        handlebox.add_artist(patch)
        return patch



def make_legend_arrow(legend, orig_handle,
                      xdescent, ydescent,
                      width, height, fontsize):

    p = mpatches.FancyArrow(0,
                            0.01,
                            15,
                            0,
                            head_width=5,
                            head_length=5,
                           )
    
    return p

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

csv_filename = '11_15_scurved_-15_-1_30'
csv_folder = 'C:/Users/wamgemls/OneDrive/Hochschule/Master/Masterarbeit/Logging/processed/csv/'
csv_filepath = csv_folder + csv_filename + '/'

contents = os.listdir(csv_folder)
subfolders = [os.path.basename(item) for item in contents if os.path.isdir(os.path.join(csv_folder, item))]

#print(subfolders)

timestamp_l = []
ego_x_l = []
ego_y_l = []
ego_vx_l = []
ego_ax_l = []
ego_yaw_l = []
ego_curvature_l = []
prekingpin_x_l = []
prekingpin_y_l = []
prekingpin_yaw_l = []
kingpin_x_l = []
kingpin_y_l = []
kingpin_yaw_l = []
dis_error_trajectory_l = []
yaw_error_trajectory_l = []
dis_error_trajectory_goal_l = []
yaw_error_trajectory_goal_l = []
dis_error_prekingpin_l = []
yaw_error_prekingpin_l = []
dis_error_kingpin_l = []
yaw_error_kingpin_l = []

trajectory_list = []
trajectory23_list = []

plt.rc('text', usetex=True)
plt.rc('font', family='serif',size=15)


folder_index=3

# Read in the CSV file and extract the timestamp and x value from each row

for folder_index in range(3,4):
    with open(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'.csv', mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        init_time = next(reader)[0]
        #print(init_time)
        file.seek(0)
        next(reader)

        timestamp_l.clear()
        ego_x_l.clear()
        ego_y_l.clear()
        ego_vx_l.clear()
        ego_ax_l.clear()
        ego_yaw_l.clear()
        ego_curvature_l.clear()
        prekingpin_x_l.clear()
        prekingpin_y_l.clear()
        prekingpin_yaw_l.clear()
        kingpin_x_l.clear()
        kingpin_y_l.clear()
        kingpin_yaw_l.clear()
        dis_error_trajectory_l.clear()
        yaw_error_trajectory_l.clear()
        dis_error_trajectory_goal_l.clear()
        yaw_error_trajectory_goal_l.clear()
        dis_error_prekingpin_l.clear()
        yaw_error_prekingpin_l.clear()
        dis_error_kingpin_l.clear()
        yaw_error_kingpin_l.clear()

        trajectory_list.clear()
        trajectory23_list.clear()


        j=0
        for j,row in enumerate(reader):
            timestamp,ego_x,ego_y,ego_yaw,ego_vx,ego_curvature,prekingpin_x,prekingpin_y,prekingpin_yaw,kingpin_x,kingpin_y,kingpin_yaw,dis_error_trajectory,yaw_error_trajectory,dis_error_trajectory_goal, yaw_error_trajectory_goal,dis_error_prekingpin, yaw_error_prekingpin,dis_error_kingpin,yaw_error_kingpin = row[:20]

            timestamp_l.append(float(timestamp)-float(init_time))
            ego_x_l.append(float(ego_x))
            ego_y_l.append(float(ego_y))
            ego_yaw_l.append(float(ego_yaw))
            ego_vx_l.append(float(ego_vx))
            ego_curvature_l.append(float(ego_curvature))
            prekingpin_x_l.append(float(prekingpin_x))
            prekingpin_y_l.append(float(prekingpin_y))
            prekingpin_yaw_l.append(float(prekingpin_yaw))
            kingpin_x_l.append(float(kingpin_x))
            kingpin_y_l.append(float(kingpin_y))
            kingpin_yaw_l.append(float(kingpin_yaw))
            dis_error_trajectory_l.append(float(dis_error_trajectory))
            yaw_error_trajectory_l.append(float(yaw_error_trajectory))
            dis_error_trajectory_goal_l.append(float(dis_error_trajectory_goal))
            yaw_error_trajectory_goal_l.append(float(yaw_error_trajectory_goal))
            dis_error_prekingpin_l.append(float(dis_error_prekingpin))
            yaw_error_prekingpin_l.append(float(yaw_error_prekingpin))
            dis_error_kingpin_l.append(float(dis_error_kingpin))
            yaw_error_kingpin_l.append(float(yaw_error_kingpin))
            
            trajectory_delimiter=[]
            for i,col in enumerate(row):
                if col == "|":
                    trajectory_delimiter.append(i+1)
            
            trajectory23_seq = row[trajectory_delimiter[0]:trajectory_delimiter[1]]
            trajectory_seq = row[trajectory_delimiter[1]:len(row)]

            trajectory23_t = []
            trajectory23_s = []
            trajectory23_x = []
            trajectory23_y = []
            trajectory23_vx = []
            trajectory23_ax = []
            trajectory23_yaw = []
            trajectory23_curvature = []

            i=0

            for i in np.arange(0,len(trajectory23_seq)-1,8):
                trajectory23_t.append(float(trajectory23_seq[i]))
                trajectory23_s.append(float(trajectory23_seq[i+1]))
                trajectory23_x.append(float(trajectory23_seq[i+2]))
                trajectory23_y.append(float(trajectory23_seq[i+3]))
                trajectory23_vx.append(float(trajectory23_seq[i+4]))
                trajectory23_ax.append(float(trajectory23_seq[i+5]))
                trajectory23_yaw.append(float(trajectory23_seq[i+6]))
                trajectory23_curvature.append(float(trajectory23_seq[i+7]))
            
            trajectory23_list.append([  timestamp,
                                        trajectory23_t,trajectory23_s,trajectory23_x,trajectory23_y,
                                        trajectory23_vx,trajectory23_ax,trajectory23_yaw,trajectory23_curvature
                                        ])
                                    
            trajectory_t = []
            trajectory_s = []
            trajectory_x = []
            trajectory_y = []
            trajectory_vx = []
            trajectory_ax = []
            trajectory_yaw = []
            trajectory_curvature = []

            i=0
            for i in np.arange(0,len(trajectory_seq)-1,8):
                trajectory_t.append(float(trajectory_seq[i]))
                trajectory_s.append(float(trajectory_seq[i+1]))
                trajectory_x.append(float(trajectory_seq[i+2]))
                trajectory_y.append(float(trajectory_seq[i+3]))
                trajectory_vx.append(float(trajectory_seq[i+4]))
                trajectory_ax.append(float(trajectory_seq[i+5]))
                trajectory_yaw.append(float(trajectory_seq[i+6]))
                trajectory_curvature.append(float(trajectory_seq[i+7]))

            trajectory_list.append([timestamp,
                                    trajectory_t,trajectory_s,trajectory_x,trajectory_y,
                                    trajectory_vx,trajectory_ax,trajectory_yaw,trajectory_curvature
                                    ])



    tr_nr = 50

    planning_scenario = plt.figure()
    gs = gridspec.GridSpec(nrows=4, ncols=2)
    ax0 = planning_scenario.add_subplot(gs[0:4, 0])    

    ax1 = planning_scenario.add_subplot(gs[0, 1])
    plt.setp(ax1.get_xticklabels(), visible=False)

    ax2 = planning_scenario.add_subplot(gs[1, 1], sharex=ax1)

    plt.setp(ax2.get_xticklabels(), visible=False)

    ax3 = planning_scenario.add_subplot(gs[2, 1], sharex=ax1)

    plt.setp(ax3.get_xticklabels(), visible=False)

    ax4 = planning_scenario.add_subplot(gs[3, 1], sharex=ax1)

    

    #bird Planning scenario
    #bird_figure,ax1 = plt.subplots()
    ax0.grid()
    plot, = ax0.plot(trajectory_list[tr_nr][3], trajectory_list[tr_nr][4],zorder=3.0,color="g",label='planner trajectory')
    ax0.set_ylabel('y-axis (m)',fontsize=16)
    ax0.set_xlabel('x-axis (m)',fontsize=16)
    

    ego_arrow_planner = patches.FancyArrow(x=trajectory_list[tr_nr][3][0],y=trajectory_list[tr_nr][4][0],
                                dx=(1/20)*trajectory_list[tr_nr][2][-1]*np.cos(trajectory_list[tr_nr][7][0]),dy=(1/20)*trajectory_list[tr_nr][2][-1]*np.sin(trajectory_list[tr_nr][7][0]),
                                head_width=(1/20)*trajectory_list[tr_nr][2][-1], head_length=(1/20)*trajectory_list[tr_nr][2][-1],fc="dimgrey", ec="k",zorder=5.0,label='ego pose')
    ax0.add_patch(ego_arrow_planner)
    goal_arrow_planner = patches.FancyArrow(x=trajectory_list[tr_nr][3][-1],y=trajectory_list[tr_nr][4][-1],
                                dx=(1/20)*trajectory_list[tr_nr][2][-1]*np.cos(trajectory_list[tr_nr][7][-1]),dy=(1/20)*trajectory_list[tr_nr][2][-1]*np.sin(trajectory_list[tr_nr][7][-1]),
                                head_width=(1/20)*trajectory_list[tr_nr][2][-1], head_length=(1/20)*trajectory_list[tr_nr][2][-1],fc="lightgrey", ec="k",zorder=4.0)
    ax0.add_patch(goal_arrow_planner)

    yaw_line_ego = ax0.axline((trajectory_list[tr_nr][3][0],trajectory_list[tr_nr][4][0]), slope=np.tan(trajectory_list[tr_nr][7][0]), color="k", linestyle=(0, (5, 5)),linewidth=0.5)
    yaw_line_goal = ax0.axline((trajectory_list[tr_nr][3][-1],trajectory_list[tr_nr][4][-1]), slope=np.tan(trajectory_list[tr_nr][7][-1]), color="k", linestyle=(0, (5, 5)),linewidth=0.5)
    lat_line_goal = ax0.axline((trajectory_list[tr_nr][3][-1],trajectory_list[tr_nr][4][-1]),(trajectory_list[tr_nr][3][-1]+3*np.cos(trajectory_list[tr_nr][7][0]+(np.pi/2)),trajectory_list[tr_nr][4][-1]+3*np.sin(trajectory_list[tr_nr][7][0]+(np.pi/2))), color="k", linestyle=(0, (5, 5)),linewidth=0.5)

    in_x,in_y = find_intersection(trajectory_list[tr_nr][3][-1],trajectory_list[tr_nr][3][-1]+np.cos(trajectory_list[tr_nr][7][0]+(np.pi/2)),
                                  trajectory_list[tr_nr][4][-1],trajectory_list[tr_nr][4][-1]+np.sin(trajectory_list[tr_nr][7][0]+(np.pi/2)),
                                  trajectory_list[tr_nr][3][0],trajectory_list[tr_nr][3][0]+np.cos(trajectory_list[tr_nr][7][0]),
                                  trajectory_list[tr_nr][4][0],trajectory_list[tr_nr][4][0]+np.sin(trajectory_list[tr_nr][7][0]))
    

    y_offset,= ax0.plot([trajectory_list[tr_nr][3][-1],in_x],[trajectory_list[tr_nr][4][-1],in_y],linewidth=0.9,linestyle=(0, (1, 1)))
    x_offset,= ax0.plot([trajectory_list[tr_nr][3][0],in_x],[trajectory_list[tr_nr][4][0],in_y],linewidth=0.9,linestyle=(0, (1, 1)))

    ax0.add_artist(Legend(ax0,handles=[y_offset,x_offset,plot,ego_arrow_planner,goal_arrow_planner], 
            labels=[r'$\Delta y$ goal offset',r'$\Delta x$ goal offset','planner trajectory',r'$x_{ego},y_{ego},\theta_{ego},\kappa_{ego}$',r'$x_{goal},y_{goal},\theta_{goal},\kappa_{goal}$'], 
            handler_map={mpatches.FancyArrow : HandlerPatch(patch_func=make_legend_arrow),
                        },fontsize=11))

    ego_x_str = str(round(trajectory_list[tr_nr][3][0],2))
    ego_y_str = str(round(trajectory_list[tr_nr][4][0],2))
    ego_yaw_str = str(round(trajectory_list[tr_nr][7][0],2))
    ego_curv_str = str(round(trajectory_list[tr_nr][8][0],2))

    goal_x_str = str(round(trajectory_list[tr_nr][3][-1],2))
    goal_y_str = str(round(trajectory_list[tr_nr][4][-1],2))
    goal_yaw_str = str(round(trajectory_list[tr_nr][7][-1],2))
    goal_curv_str = str(round(trajectory_list[tr_nr][8][-1],2))

    text_ego = r'$x_{ego}$= '+ego_x_str+' m\n'+r'$y_{ego}$= '+ego_y_str+' m\n'+r'$\theta_{ego}$= '+ego_yaw_str+' rad\n'+r'$\kappa_{ego}$= '+ego_curv_str + r' $\frac{1}{m}$'
    text_goal = r'$x_{goal}$= '+goal_x_str+'m\n'+r'$y_{goal}$= '+goal_y_str+' m\n'+r'$\theta_{goal}$= '+goal_yaw_str+' rad\n'+r'$\kappa_{goal}$= '+goal_curv_str + r' $\frac{1}{m}$'
    
    props = dict(boxstyle='round', facecolor='white', alpha=0.5)

    ax0.text(0.35, 0.05, text_ego, transform=ax0.transAxes, fontsize=12,verticalalignment='bottom',horizontalalignment='left', bbox=props)
    ax0.text(0.05, 0.05, text_goal, transform=ax0.transAxes, fontsize=12,verticalalignment='bottom',horizontalalignment='left', bbox=props)
    ax0.set_aspect('equal','datalim')
    
    #bird_figure.suptitle('Planning Scenario - Path')

    #graph Planning scenario
    #graph_figure,graph_axis = plt.subplots(4,1,sharex=True)

    i=0
    
        
    trajectory_vx, = ax1.plot(trajectory_list[tr_nr][2],trajectory_list[tr_nr][5], '-g')
    ax1.set_ylabel(r'$v_x$ $(\frac{m}{s})$',fontsize=16)
    ax1.grid()

    trajectory_ax, = ax2.plot(trajectory_list[tr_nr][2],trajectory_list[tr_nr][6], '-g')
    ax2.set_ylabel(r'$a_x$ $(\frac{m}{s^2})$',fontsize=16)
    ax2.grid()

    trajectory_yaw, = ax3.plot(trajectory_list[tr_nr][2],trajectory_list[tr_nr][7], '-g')
    ax3.set_ylabel(r'$\theta$ $(rad)$',fontsize=16)
    ax3.grid()

    trajectory_curv, = ax4.plot(trajectory_list[tr_nr][2],trajectory_list[tr_nr][8], '-g')
    ax4.set_ylabel(r'$\kappa$ $(\frac{1}{m})$',fontsize=16)
    ax4.set_xlabel('trajectory length (m)',fontsize=16)
    ax4.grid()

    #graph_figure.suptitle('Planning Scenario - Trajectory')
 
    
    #planning scenario combined

    #planning_scenario,planning_scenario_ax= plt.subplots(4,2)

    planning_scenario.set_size_inches(10,5)
    planning_scenario.tight_layout()




    #bird pathfollowing

    lowest_dis = np.inf
    index = 0
    i = 0
    for i,dis in enumerate(dis_error_trajectory_goal_l):

        if dis < lowest_dis:
            lowest_dis = dis
            index = i

    pathfollowing_fig,pathfollowing_axis = plt.subplots(1,2)
    planner_path, =pathfollowing_axis[0].plot(trajectory_list[tr_nr][3], trajectory_list[tr_nr][4],color="g",label="planner path")
    ego_path, =pathfollowing_axis[0].plot(ego_x_l, ego_y_l,color="cornflowerblue",label='ego path')
    pathfollowing_axis[0].grid()

    pathfollowing_axis[0].set_ylabel('y-axis (m)',fontsize=16)
    pathfollowing_axis[0].set_xlabel('x-axis (m)',fontsize=16)
    pathfollowing_axis[0].set_aspect('equal','datalim')

    ego_arrow_planner = patches.FancyArrow(x=trajectory_list[tr_nr][3][0],y=trajectory_list[tr_nr][4][0],
                                dx=(1/30)*trajectory_list[tr_nr][2][-1]*np.cos(trajectory_list[tr_nr][7][0]),dy=(1/30)*trajectory_list[tr_nr][2][-1]*np.sin(trajectory_list[tr_nr][7][0]),
                                head_width=(1/30)*trajectory_list[tr_nr][2][-1], head_length=(1/30)*trajectory_list[tr_nr][2][-1],fc="g", ec="k",zorder=4.0,label='ego pose')
    pathfollowing_axis[0].add_patch(ego_arrow_planner)
    goal_arrow_planner = patches.FancyArrow(x=trajectory_list[tr_nr][3][-1],y=trajectory_list[tr_nr][4][-1],
                                dx=(1/30)*trajectory_list[tr_nr][2][-1]*np.cos(trajectory_list[tr_nr][7][-1]),dy=(1/30)*trajectory_list[tr_nr][2][-1]*np.sin(trajectory_list[tr_nr][7][-1]),
                                head_width=(1/30)*trajectory_list[tr_nr][2][-1], head_length=(1/30)*trajectory_list[tr_nr][2][-1],fc="lightgreen", ec="k",zorder=4.0)
    pathfollowing_axis[0].add_patch(goal_arrow_planner)

    ego_arrow_ego = patches.FancyArrow(x=ego_x_l[0],y=ego_y_l[0],
                                dx=(1/30)*trajectory_list[tr_nr][2][-1]*np.cos(ego_yaw_l[0]),dy=(1/30)*trajectory_list[tr_nr][2][-1]*np.sin(ego_yaw_l[0]),
                                head_width=(1/30)*trajectory_list[tr_nr][2][-1], head_length=(1/30)*trajectory_list[tr_nr][2][-1],fc="b", ec="k",zorder=5.0,label='ego pose')
    pathfollowing_axis[0].add_patch(ego_arrow_ego)
    goal_arrow_ego = patches.FancyArrow(x=ego_x_l[index],y=ego_y_l[index],
                                dx=(1/30)*trajectory_list[tr_nr][2][-1]*np.cos(ego_yaw_l[index]),dy=(1/30)*trajectory_list[tr_nr][2][-1]*np.sin(ego_yaw_l[index]),
                                head_width=(1/30)*trajectory_list[tr_nr][2][-1], head_length=(1/30)*trajectory_list[tr_nr][2][-1],fc="cornflowerblue", ec="k",zorder=5.0)
    pathfollowing_axis[0].add_patch(goal_arrow_ego)

    pathfollowing_axis[0].add_artist(Legend(pathfollowing_axis[0],handles=[planner_path,ego_path,ego_arrow_planner,goal_arrow_planner,ego_arrow_ego,goal_arrow_ego], 
            labels=['planner path','ego path','planner start pose','planner goal pose','ego start pose','ego final pose'], 
            handler_map={mpatches.FancyArrow : HandlerPatch(patch_func=make_legend_arrow),
                        },fontsize=11))


    #pathfollowing_fig.suptitle('Path Following')
    


    #bird goal pose


    #bird_acc_figure,pathfollowing_axis[1] = plt.subplots()
    pathfollowing_axis[1].grid()

    yaw_ego = pathfollowing_axis[1].axline((ego_x_l[index]-trajectory_list[tr_nr][3][-1],ego_y_l[index]-trajectory_list[tr_nr][4][-1]), slope=np.tan(ego_yaw_l[index]), color="cornflowerblue", linestyle=(0, (5, 5)))
    yaw_goal =pathfollowing_axis[1].axline((trajectory_list[tr_nr][3][-1]-trajectory_list[tr_nr][3][-1],trajectory_list[tr_nr][4][-1]-trajectory_list[tr_nr][4][-1]), slope=np.tan(trajectory_list[tr_nr][7][-1]), color="g", linestyle=(0, (5, 5)))

    distance, = pathfollowing_axis[1].plot([ego_x_l[index]-trajectory_list[tr_nr][3][-1],trajectory_list[tr_nr][3][-1]-trajectory_list[tr_nr][3][-1]], [ego_y_l[index]-trajectory_list[tr_nr][4][-1],trajectory_list[tr_nr][4][-1]-trajectory_list[tr_nr][4][-1]],color="lightcoral")

    ego_arrow_planner_2 = patches.FancyArrow(x=ego_x_l[index]-trajectory_list[tr_nr][3][-1],y=ego_y_l[index]-trajectory_list[tr_nr][4][-1],
                                dx=(1/8)*dis*np.cos(ego_yaw_l[index]),dy=(1/8)*dis*np.sin(ego_yaw_l[index]),
                                head_width=(1/8)*dis, head_length=(1/8)*dis,fc="dimgrey", ec="k",zorder=5.0,label='ego pose')
    pathfollowing_axis[1].add_patch(ego_arrow_planner_2)
    goal_arrow_planner_2 = patches.FancyArrow(x=trajectory_list[tr_nr][3][-1]-trajectory_list[tr_nr][3][-1],y=trajectory_list[tr_nr][4][-1]-trajectory_list[tr_nr][4][-1],
                                dx=(1/8)*dis*np.cos(trajectory_list[tr_nr][7][-1]),dy=(1/8)*dis*np.sin(trajectory_list[tr_nr][7][-1]),
                                head_width=(1/8)*dis, head_length=(1/8)*dis,fc="lightgrey", ec="k",zorder=4.0)
    pathfollowing_axis[1].add_patch(goal_arrow_planner_2)

    pathfollowing_axis[1].set_ylabel('y-axis (m)',fontsize=16)
    pathfollowing_axis[1].set_xlabel('x-axis (m)',fontsize=16)
    pathfollowing_axis[1].set_aspect('equal','datalim')

    pathfollowing_axis[1].add_artist(Legend(pathfollowing_axis[1],handles=[distance,yaw_ego,yaw_goal,ego_arrow_planner_2,goal_arrow_planner_2], 
            labels=[r'$\Vert d_{ego2goal} \Vert_2$',r'$\theta_{ego}$',r'$\theta_{goal}$',r'$x_{ego},y_{ego},\theta_{ego}$',r'$x_{goal},y_{goal},\theta_{goal}$'], 
            handler_map={mpatches.FancyArrow : HandlerPatch(patch_func=make_legend_arrow),
                        },fontsize=11))
    
    pathfollowing_fig.set_size_inches(10,5)
    pathfollowing_fig.tight_layout()
    
    #axis.text(poseA[0],poseA[1]+0.25,r'$(x_0,y_0,\theta_0,\kappa_0)$',horizontalalignment='center',fontsize=13)

    ego_x_str = str(round(ego_x_l[index]-trajectory_list[tr_nr][3][-1],2))
    ego_y_str = str(round(ego_y_l[index]-trajectory_list[tr_nr][4][-1],2))
    ego_yaw_str = str(round(ego_yaw_l[index],2))
   

    goal_x_str = str(round(trajectory_list[tr_nr][3][-1]-trajectory_list[tr_nr][3][-1],2))
    goal_y_str = str(round(trajectory_list[tr_nr][4][-1]-trajectory_list[tr_nr][4][-1],2))
    goal_yaw_str = str(round(trajectory_list[tr_nr][7][-1],2))


    text_2_ego = r'$x_{ego}$= '+ego_x_str+' m\n'+r'$y_{ego}$= '+ego_y_str+' m\n'+r'$\theta_{ego}$= '+ego_yaw_str+' rad'
    text_2_goal = r'$x_{goal}$= '+goal_x_str+'m\n'+r'$y_{goal}$= '+goal_y_str+' m\n'+r'$\theta_{goal}$= '+goal_yaw_str+' rad'
    
    props_2 = dict(boxstyle='round', facecolor='white', alpha=0.5)

    pathfollowing_axis[1].text(0.95, 0.05, text_2_ego, transform=pathfollowing_axis[1].transAxes, fontsize=12,verticalalignment='bottom',horizontalalignment='right', bbox=props_2)
    pathfollowing_axis[1].text(0.95, 0.22, text_2_goal, transform=pathfollowing_axis[1].transAxes, fontsize=12,verticalalignment='bottom',horizontalalignment='right', bbox=props_2)

    #bird_acc_figure.suptitle('Goal Pose Situation')

    #yaw pathfollowing
    pathfollowing_yaw_fig,pathfollowing_yaw_axis = plt.subplots()
    pathfollowing_yaw_axis.plot(timestamp_l, ego_yaw_l,color="cornflowerblue",label=r'ego $\theta$ $(rad)$')
    pathfollowing_yaw_axis.grid()

    pathfollowing_yaw_axis.set_ylabel(r'yaw $\theta$ $(rad)$',fontsize=16)
    pathfollowing_yaw_axis.set_xlabel('time (s)',fontsize=16)
    #pathfollowing_yaw_fig.legend()
    pathfollowing_yaw_fig.set_size_inches(4,2)
    #pathfollowing_yaw_fig.suptitle('Path Following - Yaw over time')
    #pathfollowing_yaw_fig.tight_layout()

    #vx pathfollowing
    pathfollowing_vx_fig,pathfollowing_vx_axis = plt.subplots()
    pathfollowing_vx_axis.plot(timestamp_l, ego_vx_l,color="cornflowerblue",label=r'ego $v_x$ $(\frac{m}{s})$')
    pathfollowing_vx_axis.grid()

    pathfollowing_vx_axis.set_ylabel(r'velocity $v_x$ $(\frac{m}{s})$',fontsize=16)
    pathfollowing_vx_axis.set_xlabel('time (s)',fontsize=16)
    #pathfollowing_vx_fig.legend()
    
    pathfollowing_vx_fig.set_size_inches(4,2)
    #pathfollowing_vx_fig.suptitle('Path Following - Velocity over time')
    #pathfollowing_vx_fig.tight_layout()

    #curvature pathfollowing
    pathfollowing_curvature_fig,pathfollowing_curvature_axis = plt.subplots()
    pathfollowing_curvature_axis.plot(timestamp_l, ego_curvature_l,color="cornflowerblue",label=r'ego $\kappa$ $(\frac{1}{m})$')
    pathfollowing_curvature_axis.grid()

    pathfollowing_curvature_axis.set_ylabel(r'curvature $\kappa$ $(\frac{1}{m})$',fontsize=16)
    pathfollowing_curvature_axis.set_xlabel('time (s)',fontsize=16)
    #pathfollowing_curvature_fig.legend()
    pathfollowing_curvature_fig.set_size_inches(4,2)
    #pathfollowing_curvature_fig.suptitle('Path Following - Curvature over time')
    #pathfollowing_curvature_fig.tight_layout()

    #error_dis_goal pathfollowing
    error_dis_goal_fig,error_dis_goal_axis = plt.subplots()
    error_dis_goal_axis.plot(timestamp_l, dis_error_trajectory_goal_l,color="lightcoral")
    error_dis_goal_axis.grid()

    error_dis_goal_axis.set_ylabel(r'control error goal $y_e$ (m)',fontsize=16)
    error_dis_goal_axis.set_xlabel('time (s)',fontsize=16)
    error_dis_goal_fig.suptitle('Control error - Euclidean difference to goal trajectory point')

    #error_theta_goal pathfollowing
    error_theta_goal_fig,error_theta_goal_axis = plt.subplots()
    error_theta_goal_axis.plot(timestamp_l, np.rad2deg(yaw_error_trajectory_goal_l),color="lightcoral")
    error_theta_goal_axis.grid()

    error_theta_goal_axis.set_ylabel(r'control error goal $\theta_e$ (°)',fontsize=16)
    error_theta_goal_axis.set_xlabel('time (s)',fontsize=16)
    error_theta_goal_fig.suptitle('Control error - Angular difference to goal trajectory point')



    #error_dis pathfollowing
    error_dis_fig,error_dis_axis = plt.subplots()
    error_dis_axis.plot(timestamp_l, dis_error_trajectory_l,color="lightcoral")
    error_dis_axis.grid()

    error_dis_axis.set_ylabel(r'control error $y_e$ (m)',fontsize=16)
    error_dis_axis.set_xlabel('time (s)',fontsize=16)
    error_dis_fig.suptitle('Control error - Euclidean difference to closest trajectory point')


    #error_theta pathfollowing
    error_theta_fig,error_theta_axis = plt.subplots()
    error_theta_axis.plot(timestamp_l, np.rad2deg(yaw_error_trajectory_l),color="lightcoral")
    error_theta_axis.grid()

    error_theta_axis.set_ylabel(r'control error $\theta_e$ (°)',fontsize=16)
    error_theta_axis.set_xlabel('time (s)',fontsize=16)
    error_theta_fig.suptitle('Control error - Angular difference to closest trajectory point')

    #combi plot error goal
    error_goal_fig, error_goal_axis = plt.subplots(1,2)
    error_goal_axis[0].plot(timestamp_l, dis_error_trajectory_goal_l,color="lightcoral")
    error_goal_axis[0].grid()
    error_goal_axis[0].set_ylabel(r'control error goal $y_e$ (m)',fontsize=16)
    error_goal_axis[0].set_xlabel('time (s)',fontsize=16)

    error_goal_axis[1].plot(timestamp_l, np.rad2deg(yaw_error_trajectory_goal_l),color="lightcoral")
    error_goal_axis[1].set_ylabel(r'control error goal $\theta_e$ (°)',fontsize=16)
    error_goal_axis[1].set_xlabel('time (s)',fontsize=16)
    error_goal_axis[1].grid()

    
    error_goal_fig.set_size_inches(8,4)
    error_goal_fig.tight_layout()

    #combi plot error
    error_fig, error_axis = plt.subplots(1,2)
    error_axis[0].plot(timestamp_l, dis_error_trajectory_l,color="lightcoral")
    error_axis[0].grid()
    error_axis[0].set_ylabel(r'control error $y_e$ (m)',fontsize=16)
    error_axis[0].set_xlabel('time (s)',fontsize=16)

    error_axis[1].plot(timestamp_l, np.rad2deg(yaw_error_trajectory_l),color="lightcoral")
    error_axis[1].set_ylabel(r'control error $\theta_e$ (°)',fontsize=16)
    error_axis[1].set_xlabel('time (s)',fontsize=16)
    error_axis[1].grid()
    
    
    error_fig.set_size_inches(8,4)
    
    error_fig.tight_layout()

    
    #plt.tight_layout()
    if True:
        planning_scenario.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'planning_scenario_path.png', dpi=300,bbox_inches="tight")
        #graph_figure.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'planning_scenario_trajectory.png', dpi=300,bbox_inches="tight")
        pathfollowing_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'pathfollowing.png', dpi=300,bbox_inches="tight")
        #bird_acc_figure.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'goal_situation.png', dpi=300,bbox_inches="tight")
        pathfollowing_yaw_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'pathfollowing_yaw.png', dpi=300,bbox_inches="tight")
        pathfollowing_vx_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'pathfollowing_vx.png', dpi=300,bbox_inches="tight")
        pathfollowing_curvature_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'pathfollowing_curvature.png', dpi=300,bbox_inches="tight")
        #error_dis_goal_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'error_dis_goal.png', dpi=300,bbox_inches="tight")
        #error_theta_goal_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'error_theta_goal.png', dpi=300,bbox_inches="tight")
        #error_dis_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'error_dis.png', dpi=300,bbox_inches="tight")
        #error_theta_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'error_theta.png', dpi=300,bbox_inches="tight")
        error_goal_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'error_goal.png', dpi=300,bbox_inches="tight")
        error_fig.savefig(csv_folder+subfolders[folder_index]+'/'+subfolders[folder_index]+'_'+'error.png', dpi=300,bbox_inches="tight")



    print("path length ", trajectory_list[tr_nr][2][-1], "time length: ", trajectory_list[tr_nr][1][-1])

    plt.show()





