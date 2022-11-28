import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
from IPython import embed

robots = ["rh5_single_leg",
          "rh5_legs",
          "rh5",
          "rh5v2"]
robot_models = ["pinocchio",
                "hyrodyn",
                "rbdl",
                "kdl"]
scenes = ["vel",
          "acc",
          "acc_reduced"]
solvers = ["qpoases",
           "eiquadprog",
           "qpswift",
           "proxqp"]

if(len(sys.argv) < 2):
    raise Exception("Invalid number of command line args. Usage: plot_results.py <folder_name>")
folder = sys.argv[1]

def getAvgSceneUpdateTime(csv_list, robot, robot_model, scene):
    n_data = len(csv_list[robot+"_"+scene+"_"+robot_model+"_"+"qpoases"]["scene_update"].values)
    avg = np.array([0]*n_data)
    for s in solvers:
        key = robot+"_"+scene+"_"+robot_model+"_"+s
        avg = avg + csv_list[key]["scene_update"].values
    avg = avg/len(solvers)
    return avg

def plotSceneUpdateTime(csv_list, wbc_type, title):
    fig = plt.figure()
    plt.title(title, fontsize = 20)
    ax = fig.add_subplot(111)
    width = 0.1

    data_pinocchio      = [getAvgSceneUpdateTime(csv_list, r, "pinocchio", wbc_type) for r in robots]
    data_kdl            = [getAvgSceneUpdateTime(csv_list, r, "kdl", wbc_type) for r in robots]
    data_rbdl           = [getAvgSceneUpdateTime(csv_list, r, "rbdl", wbc_type) for r in robots]
    data_hyrodyn        = [getAvgSceneUpdateTime(csv_list, r, "hyrodyn", wbc_type) for r in robots]

    positions_group_pinocchio      = np.array(range(len(data_pinocchio)))-width*1.5
    positions_group_kdl            = np.array(range(len(data_kdl)))-width*0.5
    positions_group_rbdl           = np.array(range(len(data_rbdl)))+width*0.5
    positions_group_hyrodyn        = np.array(range(len(data_hyrodyn)))+width*1.5

    data = [data_pinocchio,data_kdl,data_rbdl,data_hyrodyn]
    positions = [positions_group_pinocchio,positions_group_kdl,positions_group_rbdl,positions_group_hyrodyn]

    bplots = []
    for d,p in zip(data,positions):
        bplots.append(plt.boxplot(d, positions=p, widths=width,showfliers=False, patch_artist=True))

    plt.xticks(list(range(len(robots))), robots, fontsize=20)
    plt.grid(True)

    colors = ['blue','orange','green','red','grey']
    for bp,c in zip(bplots,colors):
        for box in bp['boxes']:
            box.set_facecolor(color=c)
        for m in bp['medians']:
            m.set_color(color='black')

    ax.legend([bp["boxes"][0] for bp in bplots], robot_models, loc='upper left', fontsize=20)
    plt.xlabel("Robots", fontsize=30, labelpad=20)
    plt.ylabel("Time [ms]", fontsize=30, labelpad=20)

def getAvgSceneSolveTime(csv_list, robot, solver, scene):
    n_data = len(csv_list[robot+"_"+scene+"_"+"pinocchio"+"_"+solver]["scene_update"  ].values)
    avg = np.array([0]*n_data)
    tmp = ["pinocchio"]
    for rm in tmp:
        key = robot+"_"+scene+"_"+rm+"_"+solver
        avg = avg + csv_list[key]["scene_solve"].values
    avg = avg/len(tmp)
    return avg

def plotSceneSolveTime(csv_list, wbc_type, title):
    fig = plt.figure()
    plt.title(title, fontsize = 20)
    ax = fig.add_subplot(111)
    width = 0.1

    data_qpoases      = [getAvgSceneSolveTime(csv_list, r, "qpoases", wbc_type) for r in robots]
    data_eiquadprog   = [getAvgSceneSolveTime(csv_list, r, "eiquadprog", wbc_type) for r in robots]
    data_qpswift      = [getAvgSceneSolveTime(csv_list, r, "qpswift", wbc_type) for r in robots]
    data_proxqp       = [getAvgSceneSolveTime(csv_list, r, "proxqp", wbc_type) for r in robots]

    positions_group_qpoases      = np.array(range(len(data_qpoases)))-width*1.5
    positions_group_eiquadprog   = np.array(range(len(data_eiquadprog)))-width*0.5
    positions_group_qpswift      = np.array(range(len(data_qpswift)))+width*0.5
    positions_group_proxqp      = np.array(range(len(data_proxqp)))+width*1.5

    data = [data_qpoases,data_eiquadprog,data_qpswift,data_proxqp]
    positions = [positions_group_qpoases,positions_group_eiquadprog,positions_group_qpswift,positions_group_proxqp]

    bplots = []
    for d,p in zip(data,positions):
        bplots.append(plt.boxplot(d, positions=p, widths=width,showfliers=False, patch_artist=True))

    plt.xticks(list(range(len(robots))), robots, fontsize=20)
    plt.grid(True)

    colors = ['blue','orange','green','red','grey']
    for bp,c in zip(bplots,colors):
        for box in bp['boxes']:
            box.set_facecolor(color=c)
        for m in bp['medians']:
            m.set_color(color='black')

    ax.legend([bp["boxes"][0] for bp in bplots], ['QPOases','Eiquadprog', 'QPSwift', 'ProxQP'], loc='upper left', fontsize=20)
    plt.xlabel("Robots", fontsize=30, labelpad=20)
    plt.ylabel("Time [ms]", fontsize=30, labelpad=20)

def compareSerialVsHybrid(data1, data2, labels, title):
    fig = plt.figure()
    plt.title(title, fontsize = 20)
    ax = fig.add_subplot(111)
    width = 0.2

    positions_group1 = np.array(range(len(data1)))-width/2
    positions_group2 = np.array(range(len(data2)))+width/2

    bplot1 = plt.boxplot(data1,
                         positions=positions_group1,
                         widths=width,
                         showfliers=False,
                         patch_artist=True)
    bplot2 = plt.boxplot(data2,
                         positions=positions_group2,
                         widths=width,
                         showfliers=False,
                         patch_artist=True)

    plt.xticks(list(range(len(labels))) , labels, fontsize=20)
    plt.grid(True)
    for box in bplot1['boxes']:
        box.set_facecolor(color='blue')
    for box in bplot2['boxes']:
        box.set_facecolor(color='orange')
    for m in bplot1['medians']:
        m.set_color(color='black')
    ax.legend([bplot1["boxes"][0], bplot2["boxes"][0]], ['Serial Model', 'Hybrid Model'], loc='upper left', fontsize=20)
    plt.xlabel("Robot Model / WBC Type", fontsize=30, labelpad=20)
    plt.ylabel("Time per Cycle [ms]", fontsize=30, labelpad=20)

def compareScenes(csv_list, title):
    fig = plt.figure()
    plt.title(title, fontsize = 20)
    ax = fig.add_subplot(111)
    width = 0.1

    data_qpoases_vel  = [getAvgSceneUpdateTime(csv_list, r, "pinocchio", "vel") + getAvgSceneSolveTime(csv_list, r, "qpoases", "vel") for r in robots]
    data_qpoases_acc  = [getAvgSceneUpdateTime(csv_list, r, "pinocchio", "acc") + getAvgSceneSolveTime(csv_list, r, "qpoases", "acc") for r in robots]
    data_qpoases_accr = [getAvgSceneUpdateTime(csv_list, r, "pinocchio", "acc_reduced") + getAvgSceneSolveTime(csv_list, r, "qpoases", "acc_reduced") for r in robots]

    positions_group_vel   = np.array(range(len(data_qpoases_vel)))-width
    positions_group_acc   = np.array(range(len(data_qpoases_acc)))
    positions_group_accr  = np.array(range(len(data_qpoases_accr)))+width

    data = [data_qpoases_vel,data_qpoases_acc,data_qpoases_accr]
    positions = [positions_group_vel,positions_group_acc,positions_group_accr]

    bplots = []
    for d,p in zip(data,positions):
        bplots.append(plt.boxplot(d, positions=p, widths=width,showfliers=False, patch_artist=True))

    plt.xticks(list(range(len(robots))), robots, fontsize=20)
    plt.grid(True)

    colors = ['blue','orange','green','red','grey']
    for bp,c in zip(bplots,colors):
        for box in bp['boxes']:
            box.set_facecolor(color=c)
        for m in bp['medians']:
            m.set_color(color='black')

    ax.legend([bp["boxes"][0] for bp in bplots], ['Velocity','TSID', 'TSID reduced'], loc='upper left', fontsize=20)
    plt.xlabel("Robots", fontsize=30, labelpad=20)
    plt.ylabel("Time [ms]", fontsize=30, labelpad=20)

def plotScenes(robots, robot_models, solvers, scenes):
    csv_list = {}
    # load all csv files
    for r in robots:
        for b in scenes:
            for rm in robot_models:
                for s in solvers:
                     key = r + "_" + b + "_" + rm + "_" + s
                     csv_list[key] = pd.read_csv(folder + "/" + key + ".csv",sep=" ")


    # Plot update time for different robot models / robots
    #compareScenes(csv_list, "Scene Update")
    plotSceneUpdateTime(csv_list, "vel", "Scene Update (Velocity)")
    plotSceneUpdateTime(csv_list, "acc", "Scene Update (Acceleration)")

    plotSceneSolveTime(csv_list, "vel", "Scene Solve (Velocity)")
    plotSceneSolveTime(csv_list, "acc", "Scene Solve (Acceleration)")
    #compareSerialVsHybrid(total_serial, total_hybrid, ["RH5v2\nVelocity", "RH5v2\nAcceleration", "RH5 One Leg\nVelocity", "RH5 One Leg\nAcceleration", "RH5 Legs\nVelocity", "RH5 Legs\nAcceleration", "RH5\nVelocity", "RH5\nAcceleration"], "")


plotScenes(robots, robot_models, solvers, scenes)
plt.show()
