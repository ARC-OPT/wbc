import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys
from IPython import embed

robots = ["rh5_single_leg",
          "rh5_legs",
          "rh5",
          "rh5v2"]
robot_models = ["robot_model_kdl",
                "robot_model_hyrodyn",
                "robot_model_hyrodyn_hybrid"]
scenes = ["vel_kdl",
          "vel_hyrodyn",
          "vel_hyrodyn_hybrid",
          "acc_kdl",
          "acc_hyrodyn",
          "acc_hyrodyn_hybrid"]

if(len(sys.argv) < 2):
    raise Exception("Invalid number of command line args. Usage: plot_results.py <folder_name>")

folder = sys.argv[1]

def plotBoxWhysker(data1, data2, data3, labels, title):
    fig = plt.figure()
    plt.title(title, fontsize = 20)
    ax = fig.add_subplot(111)
    width = 0.2

    positions_group1 = np.array(range(len(data1)))-width
    positions_group2 = np.array(range(len(data2)))
    positions_group3 = np.array(range(len(data3)))+width

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
    bplot3 = plt.boxplot(data3,
                         positions=positions_group3,
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
    for m in bplot2['medians']:
        m.set_color(color='black')
    ax.legend([bplot1["boxes"][0], bplot2["boxes"][0], bplot3["boxes"][0]], ['Serial Model KDL', 'Serial Model Hyrodyn', 'Hybrid Model Hyrodyn'], loc='upper left', fontsize=20)
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

def plotScenes(robots, scenes):
    csv_list = {}
    for r in robots:
        for b in scenes:
            csv_list[r + "_" + b] = pd.read_csv(folder + r + "_" + b + ".csv",sep=" ")

    update_scene_vel_kdl = [csv_list[r + "_" + "vel_kdl"]["scene_update"].values/1000 for r in robots]
    update_scene_vel_hyrodyn = [csv_list[r + "_" + "vel_hyrodyn"]["scene_update"].values/1000 for r in robots]
    update_scene_vel_hyrodyn_hybrid = [csv_list[r + "_" + "vel_hyrodyn_hybrid"]["scene_update"].values/1000 for r in robots]
    update_scene_acc_kdl = [csv_list[r + "_" + "acc_kdl"]["scene_update"].values/1000 for r in robots]
    update_scene_acc_hyrodyn = [csv_list[r + "_" + "acc_hyrodyn"]["scene_update"].values/1000 for r in robots]
    update_scene_acc_hyrodyn_hybrid = [csv_list[r + "_" + "acc_hyrodyn_hybrid"]["scene_update"].values/1000 for r in robots]

    solve_scene_vel_kdl = [csv_list[r + "_" + "vel_kdl"]["scene_solve"].values/1000 for r in robots]
    solve_scene_vel_hyrodyn = [csv_list[r + "_" + "vel_hyrodyn"]["scene_solve"].values/1000 for r in robots]
    solve_scene_vel_hyrodyn_hybrid = [csv_list[r + "_" + "vel_hyrodyn_hybrid"]["scene_solve"].values/1000 for r in robots]
    solve_scene_acc_kdl = [csv_list[r + "_" + "acc_kdl"]["scene_solve"].values/1000 for r in robots]
    solve_scene_acc_hyrodyn = [csv_list[r + "_" + "acc_hyrodyn"]["scene_solve"].values/1000 for r in robots]
    solve_scene_acc_hyrodyn_hybrid = [csv_list[r + "_" + "acc_hyrodyn_hybrid"]["scene_solve"].values/1000 for r in robots]

    plotBoxWhysker(update_scene_vel_kdl, update_scene_vel_hyrodyn, update_scene_vel_hyrodyn_hybrid, robots, "Scene Update (Velocity)")
    plotBoxWhysker(update_scene_acc_kdl, update_scene_acc_hyrodyn, update_scene_acc_hyrodyn_hybrid, robots, "Scene Update (Acceleration)")
    plotBoxWhysker(solve_scene_vel_kdl, solve_scene_vel_hyrodyn, solve_scene_vel_hyrodyn_hybrid, robots, "Solve (Velocity)")
    plotBoxWhysker(solve_scene_acc_kdl, solve_scene_acc_hyrodyn, solve_scene_acc_hyrodyn_hybrid, robots, "Solve (Acceleration)")

    total_serial = [csv_list["rh5v2_vel_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5v2_vel_hyrodyn"]["scene_solve"].values/1000,
                    csv_list["rh5v2_acc_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5v2_acc_hyrodyn"]["scene_solve"].values/1000,
                    csv_list["rh5_single_leg_vel_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5_single_leg_vel_hyrodyn"]["scene_solve"].values/1000,
                    csv_list["rh5_single_leg_acc_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5_single_leg_acc_hyrodyn"]["scene_solve"].values/1000,
                    csv_list["rh5_legs_vel_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5_legs_vel_hyrodyn"]["scene_solve"].values/1000,
                    csv_list["rh5_legs_acc_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5_legs_acc_hyrodyn"]["scene_solve"].values/1000,
                    csv_list["rh5_vel_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5_vel_hyrodyn"]["scene_solve"].values/1000,
                    csv_list["rh5_acc_hyrodyn"]["scene_update"].values/1000 + csv_list["rh5_acc_hyrodyn"]["scene_solve"].values/1000]
    total_hybrid = [csv_list["rh5v2_vel_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5v2_vel_hyrodyn_hybrid"]["scene_solve"].values/1000,
                    csv_list["rh5v2_acc_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5v2_acc_hyrodyn_hybrid"]["scene_solve"].values/1000,
                    csv_list["rh5_single_leg_vel_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5_single_leg_vel_hyrodyn_hybrid"]["scene_solve"].values/1000,
                    csv_list["rh5_single_leg_acc_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5_single_leg_acc_hyrodyn_hybrid"]["scene_solve"].values/1000,
                    csv_list["rh5_legs_vel_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5_legs_vel_hyrodyn_hybrid"]["scene_solve"].values/1000,
                    csv_list["rh5_legs_acc_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5_legs_acc_hyrodyn_hybrid"]["scene_solve"].values/1000,
                    csv_list["rh5_vel_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5_vel_hyrodyn_hybrid"]["scene_solve"].values/1000,
                    csv_list["rh5_acc_hyrodyn_hybrid"]["scene_update"].values/1000 + csv_list["rh5_acc_hyrodyn_hybrid"]["scene_solve"].values/1000]

    compareSerialVsHybrid(total_serial, total_hybrid, ["RH5v2\nVelocity", "RH5v2\nAcceleration", "RH5 One Leg\nVelocity", "RH5 One Leg\nAcceleration", "RH5 Legs\nVelocity", "RH5 Legs\nAcceleration", "RH5\nVelocity", "RH5\nAcceleration"], "")


plotScenes(robots, scenes)
plt.show()
