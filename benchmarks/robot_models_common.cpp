#include "robot_models_common.hpp"
#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <robot_models/kdl/RobotModelKDL.hpp>

using namespace std;

namespace wbc{

RobotModelPtr makeRobotModelKUKAIiwa(const string type){
    RobotModelPtr robot_model;
    RobotModelConfig config;
    config.type = type;
    if(type == "hyrodyn"){
        config.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
        config.submechanism_file = "../../../models/kuka/hyrodyn/kuka_iiwa.yml";
        robot_model = std::make_shared<RobotModelHyrodyn>();
    }
    else{
        config.file = "../../../models/kuka/urdf/kuka_iiwa.urdf";
        robot_model = std::make_shared<RobotModelKDL>();
    }
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    return robot_model;
}

RobotModelPtr makeRobotModelRH5SingleLeg(const string type, bool hybrid_model){
    RobotModelPtr robot_model;
    RobotModelConfig config;
    config.type = type;
    if(type == "hyrodyn"){
        if(hybrid_model){
            config.file = "../../../models/rh5/urdf/rh5_single_leg_hybrid.urdf";
            config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg_hybrid.yml";
        }
        else{
            config.file = "../../../models/rh5/urdf/rh5_single_leg.urdf";
            config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_single_leg.yml";
        }
        robot_model = std::make_shared<RobotModelHyrodyn>();
    }
    else{
        config.file = "../../../models/rh5/urdf/rh5_single_leg.urdf";
        robot_model = std::make_shared<RobotModelKDL>();
    }
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    return robot_model;
}

RobotModelPtr makeRobotModelRH5Legs(const string type, bool hybrid_model){
    RobotModelPtr robot_model;
    RobotModelConfig config;
    config.type = type;
    if(type == "hyrodyn"){
        if(hybrid_model){
            config.file = "../../../models/rh5/urdf/rh5_legs_hybrid.urdf";
            config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs_hybrid.yml";
        }
        else{
            config.file = "../../../models/rh5/urdf/rh5_legs.urdf";
            config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_legs.yml";
        }
        robot_model = std::make_shared<RobotModelHyrodyn>();
    }
    else{
        config.file = "../../../models/rh5/urdf/rh5_legs.urdf";
        robot_model = std::make_shared<RobotModelKDL>();
    }
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    config.floating_base = true;
    config.world_frame_id = "world";
    config.floating_base_state = floating_base_state;
    config.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    config.contact_points.elements = {1,1};
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    return robot_model;
}


RobotModelPtr makeRobotModelRH5(const std::string type, bool hybrid_model){
    RobotModelPtr robot_model;
    RobotModelConfig config;
    config.type = type;
    if(type == "hyrodyn"){
        if(hybrid_model){
            config.file = "../../../models/rh5/urdf/rh5_hybrid.urdf";
            config.submechanism_file = "../../../models/rh5/hyrodyn/rh5_hybrid.yml";
        }
        else{
            config.file = "../../../models/rh5/urdf/rh5.urdf";
            config.submechanism_file = "../../../models/rh5/hyrodyn/rh5.yml";
        }
        robot_model = std::make_shared<RobotModelHyrodyn>();
    }
    else{
        config.file = "../../../models/rh5/urdf/rh5.urdf";
        robot_model = std::make_shared<RobotModelKDL>();
    }
    config.floating_base = true;
    config.world_frame_id = "world";
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.0, 0.0, 0.87);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    floating_base_state.time = base::Time::now();
    config.floating_base_state = floating_base_state;
    config.contact_points.names = {"LLAnkle_FT", "LRAnkle_FT"};
    config.contact_points.elements = {1,1};
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    return robot_model;
}

RobotModelPtr makeRobotModelRH5v2(const std::string type, bool hybrid_model){
    RobotModelPtr robot_model;
    RobotModelConfig config;
    config.type = type;
    const vector<string> head_and_gripper_joints = {"HeadPitch", "HeadRoll", "HeadYaw",
                                                    "GLF1Gear", "GLF1ProximalSegment", "GLF1TopSegment",
                                                    "GLF2Gear", "GLF2ProximalSegment", "GLF2TopSegment",
                                                    "GLF3Gear", "GLF3ProximalSegment", "GLF3TopSegment",
                                                    "GLF4Gear", "GLF4ProximalSegment", "GLF4TopSegment", "GLThumb",
                                                    "GRF1Gear", "GRF1ProximalSegment", "GRF1TopSegment",
                                                    "GRF2Gear", "GRF2ProximalSegment", "GRF2TopSegment"};
    if(type == "hyrodyn"){
        if(hybrid_model){
            config.file = "../../../models/rh5v2/urdf/rh5v2_hybrid.urdf";
            config.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2_hybrid.yml";
        }
        else{
            config.file = "../../../models/rh5v2/urdf/rh5v2.urdf";
            config.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";
            config.joint_blacklist = head_and_gripper_joints;
        }
        robot_model = std::make_shared<RobotModelHyrodyn>();
    }
    else{
        config.file = "../../../models/rh5v2/urdf/rh5v2.urdf";
        config.joint_blacklist = head_and_gripper_joints;
        robot_model = std::make_shared<RobotModelKDL>();

    }
    if(!robot_model->configure(config))
        throw std::runtime_error("Failed to configure RobotModelHyrodyn");
    return robot_model;
}
}
