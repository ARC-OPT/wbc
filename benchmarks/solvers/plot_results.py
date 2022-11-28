import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
from IPython import embed

robots = ["rh5_single_leg",
          "rh5_legs",
          "rh5",
          "rh5v2"]
solvers = ["qpoases",
           "eiquadprog",
           "qpswift",
           "proxqp"]
scenes = ["vel","acc"]

if(len(sys.argv) < 2):
    raise Exception("Invalid number of command line args. Usage: plot_results.py <folder_name>")
folder = sys.argv[1]

def getAvgSceneSolveTime(csv_list, robot, wbc_type, solver):
    key = robot+"_"+wbc_type+"_"+solver
    return csv_list[key]["scene_solve"].values

def plotSceneSolveTime(csv_list, wbc_type, title):
    fig = plt.figure()
    plt.title(title, fontsize = 20)
    ax = fig.add_subplot(111)
    width = 0.1

    data_qpoases      = [getAvgSceneSolveTime(csv_list, r, wbc_type, "qpoases") for r in robots]
    data_eiquadprog   = [getAvgSceneSolveTime(csv_list, r, wbc_type, "eiquadprog") for r in robots]
    data_qpswift      = [getAvgSceneSolveTime(csv_list, r, wbc_type, "qpswift") for r in robots]
    data_proxqp       = [getAvgSceneSolveTime(csv_list, r, wbc_type, "proxqp") for r in robots]

    positions_group_eiquadprog   = np.array(range(len(data_eiquadprog)))-1.5*width
    positions_group_qpoases      = np.array(range(len(data_qpoases)))-0.5*width
    positions_group_qpswift      = np.array(range(len(data_qpswift)))+0.5*width
    positions_group_proxqp       = np.array(range(len(data_proxqp)))+1.5*width

    data = [data_eiquadprog,data_qpoases,data_qpswift,data_proxqp]
    positions = [positions_group_eiquadprog,positions_group_qpoases,positions_group_qpswift,positions_group_proxqp]

    bplots = []
    for d,p in zip(data,positions):
        bplots.append(plt.boxplot(d, positions=p, widths=width,showfliers=False, patch_artist=True))

    plt.xticks(list(range(len(robots))), robots, fontsize=20)
    plt.grid(True)

    colors = ['blue','orange','green','yellow']
    for bp,c in zip(bplots,colors):
        for box in bp['boxes']:
            box.set_facecolor(color=c)
        for m in bp['medians']:
            m.set_color(color='black')

    ax.legend([bp["boxes"][0] for bp in bplots], ['Eiquadprog', 'QPOases','QPSwift','proxQP'], loc='upper left', fontsize=20)
    plt.xlabel("Robots", fontsize=30, labelpad=20)
    plt.ylabel("Time [ms]", fontsize=30, labelpad=20)

def plotAll(robots, solvers):
    csv_list = {}
    # load all csv files
    for r in robots:
        for b in scenes:
            for s in solvers:
                 key = r + "_" + b + "_" + s
                 csv_list[key] = pd.read_csv(folder + "/" + key + ".csv",sep=" ")

    plotSceneSolveTime(csv_list, "acc", "Scene Solve (Acceleration)")
    plotSceneSolveTime(csv_list, "vel", "Scene Solve (Velocity)")

plotAll(robots, solvers)
plt.show()
