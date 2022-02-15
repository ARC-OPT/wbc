import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from IPython import embed
import sys

robots = ["rh5_single_leg",
          "rh5_legs",
          "rh5",
          "rh5v2"]
robot_models = ["robot_model_kdl",
                "robot_model_hyrodyn",
                "robot_model_hyrodyn_hybrid"]

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

def plotRobotModels(robots, robot_models):
    # Load CSV
    csv_list = {}
    for r in robots:
        for b in robot_models:
            csv_list[r + "_" + b] = pd.read_csv(folder + r + "_" + b + ".csv",sep=" ")

    space_jac_kdl = [csv_list[r + "_" + "robot_model_kdl"]["space_jac"].values/1000 for r in robots]
    space_jac_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["space_jac"].values/1000 for r in robots]
    space_jac_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["space_jac"].values/1000 for r in robots]

    body_jac_kdl = [csv_list[r + "_" + "robot_model_kdl"]["body_jac"].values/1000 for r in robots]
    body_jac_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["body_jac"].values/1000 for r in robots]
    body_jac_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["body_jac"].values/1000 for r in robots]

    fk_kdl = [csv_list[r + "_" + "robot_model_kdl"]["FK"].values/1000 for r in robots]
    fk_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["FK"].values/1000 for r in robots]
    fk_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["FK"].values/1000 for r in robots]

    bias_forces_kdl = [csv_list[r + "_" + "robot_model_kdl"]["bias_forces"].values/1000 for r in robots]
    bias_forces_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["bias_forces"].values/1000 for r in robots]
    bias_forces_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["bias_forces"].values/1000 for r in robots]

    inertia_mat_kdl = [csv_list[r + "_" + "robot_model_kdl"]["joint_space_inertia_mat"].values/1000 for r in robots]
    inertia_mat_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["joint_space_inertia_mat"].values/1000 for r in robots]
    inertia_mat_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["joint_space_inertia_mat"].values/1000 for r in robots]

    com_kdl = [csv_list[r + "_" + "robot_model_kdl"]["com"].values/1000 for r in robots]
    com_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["com"].values/1000 for r in robots]
    com_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["com"].values/1000 for r in robots]

    plotBoxWhysker(space_jac_kdl, space_jac_hyrodyn, space_jac_hyrodyn_hybrid, robots, "Space Jacobian")
    plotBoxWhysker(body_jac_kdl, body_jac_hyrodyn, body_jac_hyrodyn_hybrid, robots, "Body Jacobian")
    plotBoxWhysker(fk_kdl, fk_hyrodyn, fk_hyrodyn_hybrid, robots, "Forward Kinematics (Pose,Twist,Acceleration)")
    plotBoxWhysker(bias_forces_kdl, bias_forces_hyrodyn, bias_forces_hyrodyn_hybrid, robots, "Bias Forces/Torques")
    plotBoxWhysker(inertia_mat_kdl, inertia_mat_hyrodyn, inertia_mat_hyrodyn_hybrid, robots, "Joint Space Inertia Matrix")
    plotBoxWhysker(com_kdl, com_hyrodyn, com_hyrodyn_hybrid, robots, "CoM")

plotRobotModels(robots, robot_models)
plt.show()
