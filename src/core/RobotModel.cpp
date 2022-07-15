#include "RobotModel.hpp"
#include <base-logging/Logging.hpp>
#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/samples/Joints.hpp>

namespace wbc{

std::vector<std::string> operator+(std::vector<std::string> a, std::vector<std::string> b){
    a.insert( a.end(), b.begin(), b.end() );
    return a;
}

RobotModel::RobotModel() :
    gravity(base::Vector3d(0,0,-9.81)){
}

void RobotModel::clear(){
    independent_joint_names.clear();
    joint_names.clear();
    actuated_joint_names.clear();
    joint_names_floating_base.clear();
    contact_points.clear();
    base_frame="";
    world_frame="";
    gravity = base::Vector3d(0,0,-9.81);
    has_floating_base = false;
    joint_limits.clear();
    robot_urdf.reset();
    joint_state.clear();
    joint_state_out.clear();
    space_jac_map.clear();
    body_jac_map.clear();
    jac_dot_map.clear();
}



base::Matrix3d getRotMatfromRPY(const base::Vector3d& rpy)
{
    // Returns the Rotation matrix (3x3) by applying Euler angles in the order: roll[0], pitch[1], yaw[2]
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    base::Matrix3d R_z, R_y, R_x;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw),  cos(yaw), 0,
                  0,         0, 1;
    R_y << cos(pitch), 0, sin(pitch),
                    0, 1,          0,
          -sin(pitch), 0, cos(pitch);
    R_x << 1,         0,          0,
           0, cos(roll), -sin(roll),
           0, sin(roll),  cos(roll);

    return R_z*R_y*R_x;
}

base::Vector3d getEulerAngles(const base::Quaterniond& orientation)
{
    // Returns the Euler angles in the order: roll[0], pitch[1], yaw[2]
    // Inspired from Determining yaw, pitch, and roll from a rotation matrix (http://planning.cs.uiuc.edu/node103.html) and Pose.cpp from base-types
    const Eigen::Matrix3d m = orientation.toRotationMatrix();
    //cout<<"Input RotMat from Quat: \n"<<m<<endl;
    double x = Eigen::Vector2d(m.coeff(2,2) , m.coeff(2,1)).norm();	// x = sqrt(r33^2 + r32^2)
    base::Vector3d res(0,::atan2(-m.coeff(2,0), x),0);	// pitch = atan2(-r31, x)
    if (x > Eigen::NumTraits<double>::dummy_precision()){
        res[2] = ::atan2(m.coeff(1,0), m.coeff(0,0));	// yaw = atan2(r21, r11)
        res[0] = ::atan2(m.coeff(2,1), m.coeff(2,2));	// roll = atan2(r32, r33)
    }else{
        res[2] = 0;
        res[0] = (m.coeff(2,0)>0?1:-1)* ::atan2(-m.coeff(0,1), m.coeff(1,1));
    }

    //cout<<"Output RotMat from RPY: \n"<<getRotMat(res)<<endl;
    res = -res;	// this produces correct derivatives of floating base coordinates and lower error in inverse dynamics (not clear, why?)
    return res;
}

base::Vector3d getEulerAnglesDerivative(const base::Quaterniond& orientation, const base::Vector3d& omega)
{
    // Returns the time derivative of Euler angles in the order: yaw[0], pitch[1], roll[2]

    Eigen::Vector3d rpy = getEulerAngles(orientation);
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    Eigen::Matrix3d m;
     // Method 1: this formulation is motivated from body fixed angular velocity, see https://davidbrown3.github.io/2017-07-25/EulerAngles/
    m << cos(yaw)/cos(pitch), -sin(yaw)/cos(pitch), 0,
         sin(yaw),             cos(yaw),            0,
        -cos(yaw)*sin(pitch)/cos(pitch), sin(yaw)*sin(pitch)/cos(pitch), 1;
    //m = -m; // correction term  (it produces correct derivatives of the floating base coordinates as well but error in inverse dynamics is high)

    /*	// Method 2: Inspired from Differential rotations (http://planning.cs.uiuc.edu/node690.html), corrected by Khalil's book
    m << cos(yaw)/cos(pitch), sin(yaw)/cos(pitch), 0,
         -sin(yaw),           cos(yaw),            0,
         cos(yaw)*sin(pitch)/cos(pitch), sin(yaw)*sin(pitch)/cos(pitch), 1;
    */
    // rpy_dot = A * omega, where A is the Jacobian that maps the angular velocity(omega) to RPY angular rates
    base::Vector3d rpy_dot = m*omega;

    return rpy_dot;
}


base::Vector3d getEulerAnglesSecondDerivative(const base::Quaterniond& orientation, const base::Vector3d& omega, const base::Vector3d& omega_dot)
{
    // Returns the 2nd time derivative of Euler angles in the order: yaw[0], pitch[1], roll[2]
    // 1. Extract RPY angles from Quaternion representation
    Eigen::Vector3d rpy = getEulerAngles(orientation);
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);
    // 2. Extract RPY angle rates from Quaternion and angular velocity
    base::Vector3d rpy_dot = getEulerAnglesDerivative(orientation, omega);
    double roll_dot = rpy_dot(0);
    double pitch_dot = rpy_dot(1);
    double yaw_dot = rpy_dot(2);

    // 3. Extract RPY 2nd derivative (symbolic derivative derived in MATLAB symbolic toolbox of Method 1)
    double t2 = cos(pitch);
    double t3 = cos(yaw);
    double t4 = sin(pitch);
    double t5 = sin(yaw);
    double t6 = t4*t4;
    double t7 = 1.0/t2;
    double t8 = t7*t7;
    base::Matrix3d m;
    m(0,0) = -t5*t7*yaw_dot+pitch_dot*t3*t4*t8;
    m(0,1) = -t3*t7*yaw_dot-pitch_dot*t4*t5*t8;
    m(0,2) = 0.0;
    m(1,0) = t3*yaw_dot;
    m(1,1) = -t5*yaw_dot;
    m(1,2) = 0.0;
    m(2,0) = -pitch_dot*t3-pitch_dot*t3*t6*t8+t4*t5*t7*yaw_dot;
    m(2,1) = pitch_dot*t5+pitch_dot*t5*t6*t8+t3*t4*t7*yaw_dot;
    m(2,2) = 0.0;
    //m = -m;	// correction term (it produces correct derivatives of the floating base coordinates as well but error in inverse dynamics is high)

    /*
    // additional sign changes as per lecture notes (Method 2)
    m(0,1) = -m(0,1);
    m(1,0) = -m(1,0);
    m(2,0) = -m(2,0);
    */
    // rpy_ddot = A * omega_dot + Adot * omega
    base::Vector3d rpy_ddot = getEulerAnglesDerivative(orientation, omega_dot) + m*omega;

    return rpy_ddot;
}

void RobotModel::updateFloatingBase(const base::samples::RigidBodyStateSE3& rbs,
                                    const std::vector<std::string> &floating_base_virtual_joint_names,
                                    base::samples::Joints& joint_state){

    if(floating_base_virtual_joint_names.size() != 6){
        LOG_ERROR("Size of floating base virtual joint names has to be 6 but is %i", floating_base_virtual_joint_names.size());
        throw std::runtime_error("Invalid floating base virtual joint names");
    }

    if(!rbs.hasValidPose() ||
       !rbs.hasValidTwist() ||
       !rbs.hasValidAcceleration()){
       LOG_ERROR("Invalid status of floating base given! One (or all) of pose, twist or acceleration members is invalid (Either NaN or non-unit quaternion)");
       throw std::runtime_error("Invalid floating base status");
    }

    floating_base_state = rbs;
    base::JointState js;
    base::Vector3d euler = getEulerAngles(rbs.pose.orientation);//.toRotationMatrix().eulerAngles(0, 1, 2);

    base::Vector3d omega(rbs.twist.angular[0], rbs.twist.angular[1], rbs.twist.angular[2]);
    base::Vector3d rpy_dot = getEulerAnglesDerivative(rbs.pose.orientation, omega);

    base::Vector3d omega_dot(rbs.acceleration.angular[0], rbs.acceleration.angular[1], rbs.acceleration.angular[2]);
    base::Vector3d rpy_ddot = getEulerAnglesSecondDerivative(rbs.pose.orientation, omega, omega_dot);

    for(int j = 0; j < 3; j++){
        js.position = rbs.pose.position(j);
        js.speed = rbs.twist.linear(j);
        js.acceleration = rbs.acceleration.linear(j);
        joint_state[floating_base_virtual_joint_names[j]] = js;

        js.position = euler(j);
        js.speed = rpy_dot(j);
        js.acceleration = rpy_ddot(j);
        joint_state[floating_base_virtual_joint_names[j+3]] = js;
    }

    // Set timestamp of joint state vector to floating base timestamp in case it is older
    if(rbs.time.isNull()){
        LOG_ERROR("Floating base state does not have a valid timestamp. Or do we have 1970?");
        throw std::runtime_error("Invalid call to update()");
    }

    if(rbs.time < joint_state.time)
        joint_state.time = rbs.time;
}

void RobotModel::setActiveContacts(const ActiveContacts &contacts){
    for(auto name : contacts.names){
        if(contacts[name] != 0 && contacts[name] != 1)
            throw std::runtime_error("RobotModel::setActiveContacts: Contact value has to been 0 or 1");
    }
    active_contacts = contacts;
}


uint RobotModel::jointIndex(const std::string &joint_name){
    uint idx = std::find(joint_names.begin(), joint_names.end(), joint_name) - joint_names.begin();
    if(idx >= joint_names.size())
        throw std::invalid_argument("Index of joint  " + joint_name + " was requested but this joint is not in robot model");
    return idx;
}


bool RobotModel::hasLink(const std::string &link_name){
    for(auto l  : robot_urdf->links_)
        if(l.second->name == link_name)
            return true;
    return false;
}

bool RobotModel::hasJoint(const std::string &joint_name){
    return std::find(joint_names.begin(), joint_names.end(), joint_name) != joint_names.end();
}

bool RobotModel::hasActuatedJoint(const std::string &joint_name){
    return std::find(actuated_joint_names.begin(), actuated_joint_names.end(), joint_name) != actuated_joint_names.end();
}

const base::samples::Joints& RobotModel::jointState(const std::vector<std::string> &joint_names){

    if(joint_state.time.isNull()){
        LOG_ERROR("RobotModel: You have to call update() with appropriately timestamped joint data at least once before requesting joint state information!");
        throw std::runtime_error("Invalid call to jointState()");
    }

    joint_state_out.resize(joint_names.size());
    joint_state_out.names = joint_names;
    joint_state_out.time = joint_state.time;

    for(size_t i = 0; i < joint_names.size(); i++){
        try{
            joint_state_out[i] = joint_state.getElementByName(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("RobotModel: Requested state of joint %s but this joint does not exist in robot model", joint_names[i].c_str());
            throw std::invalid_argument("Invalid call to jointState()");
        }
    }
    return joint_state_out;
}

} // namespace wbc
