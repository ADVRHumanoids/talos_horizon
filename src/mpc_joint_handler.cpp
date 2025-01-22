#include "mpc_joint_handler.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <eigen_conversions/eigen_msg.h>

#include "utils.h"

MPCJointHandler::MPCJointHandler(ros::NodeHandle nh,
                                 XBot::ModelInterface::Ptr model,
                                 int rate,
                                 YAML::Node config,
                                 XBot::RobotInterface::Ptr robot,
                                 std::map<std::string, double> fixed_joints_map):
MPCHandler(nh),
_model(model),
_robot(robot),
_rate(rate),
_flag_id(true)
{
    init_publishers_and_subscribers();

    _fixed_joints_map = fixed_joints_map;

//    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _model->getJointAcceleration(_qddot);
    _model->getJointEffort(_tau);

    auto urdf_model = std::make_shared<urdf::ModelInterface>(_robot->getUrdf());


    // get horizon duration and n_nodes from YAML file for the Resampler
    if (config["horizon_duration"])
    {
        _horizon_duration = config["horizon_duration"].as<double>();
        ColoredTextPrinter::print("Setting horizon duration to controller: " + std::to_string(_horizon_duration), ColoredTextPrinter::TextColor::Green);
    }
    else
    {
        throw std::runtime_error("Missing required parameter 'horizon_duration'!");
    }

    if (config["n_nodes"])
    {
        _n_nodes = config["n_nodes"].as<int>();
        ColoredTextPrinter::print("Setting number of nodes to controller: " + std::to_string(_n_nodes), ColoredTextPrinter::TextColor::Green);
    }
    else
    {
        throw std::runtime_error("Missing required parameter 'n_nodes'!");
    }

    _resampler = std::make_unique<Resampler>(_horizon_duration, _n_nodes, urdf_model, _fixed_joints_map);
    _resampler_pub = _nh.advertise<sensor_msgs::JointState>("/resampler_solution_position", 1, true);
}

void MPCJointHandler::mpc_joint_callback(const cogimon_controller::WBTrajectoryConstPtr msg)
{
    _mpc_solution = *msg;

    if (!_is_callback_done)
    {
        _joint_names.insert(_joint_names.begin(), std::begin(_mpc_solution.joint_names), std::end(_mpc_solution.joint_names));

//        _x.resize(_mpc_solution.q.size() + _mpc_solution.v.size());
//        _u.resize(_mpc_solution.a.size() + _mpc_solution.force_names.size() * 6);

//        _p.resize(_resampler->nq());
//        _v.resize(_resampler->nv());
//        _a.resize(_resampler->nv());
//        _f.resize(_mpc_solution.force_names.size() * 6);

//        std::vector<std::string> frames(_mpc_solution.force_names.data(), _mpc_solution.force_names.data() + _mpc_solution.force_names.size());
//        _resampler->setFrames(frames);

//        if (frames.empty())
//        {
//            _flag_id = false;
//        }
    }

    // getting the input from the MPC solution
    _p = Eigen::VectorXd::Map(_mpc_solution.q.data(), _mpc_solution.q.size());
    _v = Eigen::VectorXd::Map(_mpc_solution.v.data(), _mpc_solution.v.size());
    _a = Eigen::VectorXd::Map(_mpc_solution.a.data(), _mpc_solution.a.size());
    _tau_ff = Eigen::VectorXd::Map(_mpc_solution.tau.data(), _mpc_solution.tau.size());

//    for (int i = 0; i < _mpc_solution.force_names.size(); i++)
//    {
//        _f.block<6, 1>(i * 6, 0) << _mpc_solution.f[i].x, _mpc_solution.f[i].y, _mpc_solution.f[i].z, 0, 0, 0;
//    }

//    _x << _p, _v;
//    _u << _a, _f;

//    if(!_resampler->setState(_x))
//        throw std::runtime_error("wrong dimension of the state vector! " + std::to_string(_x.size()) + " != ");
//    if(!_resampler->setInput(_u))
//        throw std::runtime_error("wrong dimension of the input vector! " + std::to_string(_u.size()) + " != ");


    _is_callback_done = true;
//    _solution_index = 1;
}

void MPCJointHandler::init_publishers_and_subscribers()
{
    _mpc_sub = _nh.subscribe("/mpc_solution", 1, &MPCJointHandler::mpc_joint_callback, this);
}

void MPCJointHandler::setTorqueOffset(XBot::JointNameMap tau_offset)
{
    _tau_offset = tau_offset;
}

void MPCJointHandler::smooth(const Eigen::VectorXd state, const Eigen::VectorXd input, Eigen::VectorXd& out)
{
    double alpha = 0.1;
    out = alpha * input + (1-alpha) * state;
}

bool MPCJointHandler::update_no_resampler()
{
    _robot->setPositionReference(_p.tail(_robot->getJointNum()));
    _robot->setVelocityReference(_v.tail(_robot->getJointNum()));
    _robot->setEffortReference(_tau_ff.tail(_robot->getJointNum()));
    _robot->move();

    return true;
}

bool MPCJointHandler::update()
{
    _robot->sense();

//     resample
    _resampler->resample(1./_rate);

//     get resampled state and set it to the robot
    Eigen::VectorXd tau;
    _resampler->getState(_x);
    _resampler->getInput(_u);

    if (_flag_id)
    {
        _resampler->getTau(tau);
    }

    _p = _x.head(_p.size());
    _v = _x.segment(_p.size(), _v.size());
    _a = _u.head(_a.size());

    msg_pub.position.clear();
    msg_pub.velocity.clear();
    msg_pub.position.assign(_p.data(), _p.data() + _p.size());
    msg_pub.velocity.assign(_v.data(), _v.data() + _v.size());
    
    if (_flag_id)
    {
        msg_pub.effort.assign(tau.data(), tau.data() + tau.size());
    }

    _resampler_pub.publish(msg_pub);


    _resampler->getMinimalJointMap(_q);
    vectors_to_map<std::string, double>(_joint_names, _v.tail(_v.size() - 6), _qdot);


    if (_flag_id)
    {
        vectors_to_map<std::string, double>(_joint_names, tau.tail(tau.size() - 6), _tau);


        for (auto &pair : _tau)
            pair.second -= _tau_offset[pair.first];
    }

//    _robot->setPositionReference(_q);
//    _robot->setVelocityReference(_qdot);

//    if (_flag_id)
//    {
//        _robot->setEffortReference(_tau);
//    }
    
//    _robot->move();

    return true;
}
