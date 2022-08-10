import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from IPython import embed
import sys

robots = ["rh5_single_leg",
          "rh5_legs",
          "rh5",
          "rh5v2"]
robot_models = ["robot_model_pinocchio",
                "robot_model_rbdl",
                "robot_model_kdl",
                "robot_model_hyrodyn",
                "robot_model_hyrodyn_hybrid"]

if(len(sys.argv) < 2):
    raise Exception("Invalid number of command line args. Usage: plot_results.py <folder_name>")

folder = sys.argv[1]

def plotBoxWhysker(data_pinocchio, data_rbdl, data_kdl, data_hyrodyn, data_hyrodyn_hybrid, labels, title):
    fig = plt.figure()
    plt.title(title, fontsize = 20)
    ax = fig.add_subplot(111)
    width = 0.1

    positions_group_pinocchio = np.array(range(len(data_pinocchio)))-2*width
    positions_group_rbdl = np.array(range(len(data_rbdl)))-width
    positions_group_kdl = np.array(range(len(data_kdl)))
    positions_group_hyrodyn = np.array(range(len(data_hyrodyn)))+width
    positions_group_hyrodyn_hybrid = np.array(range(len(data_hyrodyn_hybrid)))+width*2

    data = [data_pinocchio, data_rbdl, data_kdl, data_hyrodyn, data_hyrodyn_hybrid]
    positions = [positions_group_pinocchio,positions_group_rbdl,positions_group_kdl,positions_group_hyrodyn,positions_group_hyrodyn_hybrid]

    bplots = []
    for d,p in zip(data,positions):
        bplots.append(plt.boxplot(d, positions=p, widths=width,showfliers=False, patch_artist=True))

    plt.xticks(list(range(len(labels))) , labels, fontsize=20)
    plt.grid(True)

    colors = ['blue','orange','green','red','grey']
    for bp,c in zip(bplots,colors):
        for box in bp['boxes']:
            box.set_facecolor(color=c)
        for m in bp['medians']:
            m.set_color(color='black')

    ax.legend([bp["boxes"][0] for bp in bplots], ['Pinocchio', 'RBDL', 'KDL', 'Hyrodyn', 'Hyrodyn Hybrid'], loc='upper left', fontsize=20)
    plt.xlabel("Robots", fontsize=30, labelpad=20)
    plt.ylabel("Time [us]", fontsize=30, labelpad=20)

def plotRobotModels(robots, robot_models):
    # Load CSV
    csv_list = {}
    for r in robots:
        for b in robot_models:
            csv_list[r + "_" + b] = pd.read_csv(folder + r + "_" + b + ".csv",sep=" ")

    space_jac_pinocchio = [csv_list[r + "_" + "robot_model_pinocchio"]["space_jac"].values*1000 for r in robots]
    space_jac_rbdl = [csv_list[r + "_" + "robot_model_rbdl"]["space_jac"].values*1000 for r in robots]
    space_jac_kdl = [csv_list[r + "_" + "robot_model_kdl"]["space_jac"].values*1000 for r in robots]
    space_jac_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["space_jac"].values*1000 for r in robots]
    space_jac_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["space_jac"].values*1000 for r in robots]

    body_jac_pinocchio = [csv_list[r + "_" + "robot_model_pinocchio"]["body_jac"].values*1000 for r in robots]
    body_jac_rbdl = [csv_list[r + "_" + "robot_model_rbdl"]["body_jac"].values*1000 for r in robots]
    body_jac_kdl = [csv_list[r + "_" + "robot_model_kdl"]["body_jac"].values*1000 for r in robots]
    body_jac_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["body_jac"].values*1000 for r in robots]
    body_jac_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["body_jac"].values*1000 for r in robots]


    fk_pinocchio = [csv_list[r + "_" + "robot_model_pinocchio"]["FK"].values*1000 for r in robots]
    fk_rbdl = [csv_list[r + "_" + "robot_model_rbdl"]["FK"].values*1000 for r in robots]
    fk_kdl = [csv_list[r + "_" + "robot_model_kdl"]["FK"].values*1000 for r in robots]
    fk_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["FK"].values*1000 for r in robots]
    fk_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["FK"].values*1000 for r in robots]

    bias_forces_pinocchio = [csv_list[r + "_" + "robot_model_pinocchio"]["bias_forces"].values*1000 for r in robots]
    bias_forces_rbdl = [csv_list[r + "_" + "robot_model_rbdl"]["bias_forces"].values*1000 for r in robots]
    bias_forces_kdl = [csv_list[r + "_" + "robot_model_kdl"]["bias_forces"].values*1000 for r in robots]
    bias_forces_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["bias_forces"].values*1000 for r in robots]
    bias_forces_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["bias_forces"].values*1000 for r in robots]

    inertia_mat_pinocchio = [csv_list[r + "_" + "robot_model_pinocchio"]["joint_space_inertia_mat"].values*1000 for r in robots]
    inertia_mat_rbdl = [csv_list[r + "_" + "robot_model_rbdl"]["joint_space_inertia_mat"].values*1000 for r in robots]
    inertia_mat_kdl = [csv_list[r + "_" + "robot_model_kdl"]["joint_space_inertia_mat"].values*1000 for r in robots]
    inertia_mat_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["joint_space_inertia_mat"].values*1000 for r in robots]
    inertia_mat_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["joint_space_inertia_mat"].values*1000 for r in robots]

    com_pinocchio = [csv_list[r + "_" + "robot_model_pinocchio"]["com"].values*1000 for r in robots]
    com_rbdl = [csv_list[r + "_" + "robot_model_rbdl"]["com"].values*1000 for r in robots]
    com_kdl = [csv_list[r + "_" + "robot_model_kdl"]["com"].values*1000 for r in robots]
    com_hyrodyn = [csv_list[r + "_" + "robot_model_hyrodyn"]["com"].values*1000 for r in robots]
    com_hyrodyn_hybrid = [csv_list[r + "_" + "robot_model_hyrodyn_hybrid"]["com"].values*1000 for r in robots]

    plotBoxWhysker(space_jac_pinocchio, space_jac_rbdl, space_jac_kdl, space_jac_hyrodyn, space_jac_hyrodyn_hybrid, robots, "Space Jacobian")
    plotBoxWhysker(body_jac_pinocchio, body_jac_rbdl, body_jac_kdl, body_jac_hyrodyn, body_jac_hyrodyn_hybrid, robots, "Body Jacobian")
    plotBoxWhysker(fk_pinocchio, fk_rbdl, fk_kdl, fk_hyrodyn, fk_hyrodyn_hybrid, robots, "Forward Kinematics (Pose,Twist,Acceleration)")
    plotBoxWhysker(bias_forces_pinocchio, bias_forces_rbdl, bias_forces_kdl, bias_forces_hyrodyn, bias_forces_hyrodyn_hybrid, robots, "Bias Forces/Torques")
    plotBoxWhysker(inertia_mat_pinocchio, inertia_mat_rbdl, inertia_mat_kdl, inertia_mat_hyrodyn, inertia_mat_hyrodyn_hybrid, robots, "Joint Space Inertia Matrix")
    plotBoxWhysker(com_pinocchio, com_rbdl, com_kdl, com_hyrodyn, com_hyrodyn_hybrid, robots, "CoM")

plotRobotModels(robots, robot_models)
plt.show()
