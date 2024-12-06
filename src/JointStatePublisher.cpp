#include "JointStatePublisher.h"

using namespace XBot;

JointStatePublisher::JointStatePublisher(ros::NodeHandle nh):
_nh(nh),
_nhpr("~")
{
    init_robot();

    _js_pub = _nh.advertise<xbot_msgs::JointState>("joint_state", 1);

    _q.resize(_robot->getJointNum());
    _q_ref.resize(_robot->getJointNum());
    _qdot.resize(_robot->getJointNum());
    _qdot_ref.resize(_robot->getJointNum());
    _stiff.resize(_robot->getJointNum());
    _damp.resize(_robot->getJointNum());
    _tau_ff.resize(_robot->getJointNum());

    _joint_names = _robot->getEnabledJointNames();
}

void JointStatePublisher::init_robot()
{
    // Create instance of ModelInferace and RobotInterface
    XBot::ConfigOptions opt;
    std::string urdf, srdf, jidmap;
    if(_nh.hasParam("robot_description") && _nh.getParam("robot_description", urdf))
    {
        opt.set_urdf(urdf);
    }
    else
    {
        throw std::runtime_error("robot_description parameter not set");
    }

    if(_nh.hasParam("robot_description_semantic") && _nh.getParam("robot_description_semantic", srdf))
    {
        opt.set_srdf(srdf);
    }
    else
    {
        throw std::runtime_error("robot_description_semantic parameter not set");
    }

    if(_nh.hasParam("robot_description_joint_id_map") && _nh.getParam("robot_description_joint_id_map", jidmap))
    {
        opt.set_jidmap(jidmap);
    }
    else
    {
        //success = false;
        if(!opt.generate_jidmap())
            throw std::runtime_error("robot_description_joint_id_map parameter not set, failed to auto-generate jid_map");
    }

    std::string model_type;
    bool is_model_floating_base;

    opt.set_parameter("model_type", _nhpr.param<std::string>("model_type", "RBDL"));
    opt.set_parameter("is_model_floating_base", _nhpr.param<bool>("is_model_floating_base", true));
    opt.set_parameter<std::string>("framework", "Unitree");

    _robot = XBot::RobotInterface::getRobot(opt);
}

void JointStatePublisher::publish()
{
    xbot_msgs::JointState xbot_msg;

    _robot->sense(false, false);

    _robot->getJointPosition(_q);
    _robot->getPositionReference(_q_ref);
    _robot->getJointVelocity(_qdot);
    _robot->getVelocityReference(_qdot_ref);
    _robot->getStiffness(_stiff);
    _robot->getDamping(_damp);
    _robot->getEffortReference(_tau_ff);

    xbot_msg.name.assign(_joint_names.begin(), _joint_names.begin() + _joint_names.size());
    xbot_msg.link_position.assign(_q.data(), _q.data() + _q.size());
    xbot_msg.position_reference.assign(_q_ref.data(), _q_ref.data() + _q_ref.size());
    xbot_msg.link_velocity.assign(_qdot.data(), _qdot.data() + _qdot.size());
    xbot_msg.stiffness.assign(_stiff.data(), _stiff.data() + _stiff.size());
    xbot_msg.damping.assign(_damp.data(), _damp.data() + _damp.size());
    xbot_msg.effort.assign(_tau_ff.data(), _tau_ff.data() + _tau_ff.size());

    _js_pub.publish(xbot_msg);
}

