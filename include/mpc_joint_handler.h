#ifndef MPC_JOINT_HANDLER_H
#define MPC_JOINT_HANDLER_H

#include "resampler.h"
#include "mpc_handler.h"
#include <talos_horizon/WBTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>

#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <sensor_msgs/JointState.h>

#include <algorithm>

class MPCJointHandler : public MPCHandler {
public:
    typedef std::shared_ptr<MPCJointHandler> Ptr;
    MPCJointHandler(ros::NodeHandle nh,
                    XBot::ModelInterface::Ptr model,
                    int rate,
                    YAML::Node config,
                    XBot::RobotInterface::Ptr robot = nullptr,
                    std::map<std::string, double> fixed_joints_map = {});

    void setTorqueOffset(XBot::JointNameMap tau_offset);

    bool update() override;
    bool update_no_resampler();

private:

    void init_publishers_and_subscribers();

    void mpc_joint_callback(const talos_horizon::WBTrajectoryConstPtr msg);

    void smooth(const Eigen::VectorXd state, const Eigen::VectorXd in, Eigen::VectorXd& out);

    template<typename key, typename value>
    void vectors_to_map(const std::vector<key> vec1, const Eigen::Matrix<value, 1, -1> vec2, std::unordered_map<key, value>& map)
    {
        if (vec1.size() != vec2.size())
        {
            throw std::runtime_error("you are trying to merge two vectors of different size in the same map! (" + std::to_string(vec1.size()) + " != " + std::to_string(vec2.size()) + ")");
        }

        for (int i = 0; i < vec1.size(); i++)
        {
            map[vec1[i]] = vec2[i];
        }
    }

    double _horizon_duration;
    int _n_nodes;

    ros::Publisher _resampler_pub;
    sensor_msgs::JointState msg_pub;

    XBot::JointNameMap _q, _qdot, _qddot, _tau;
    XBot::JointNameMap _tau_offset;

    Eigen::VectorXd _p, _v, _a, _f, _tau_ff;
    Eigen::VectorXd _j, _fdot;

    std::vector<std::string> _joint_names;
    std::map<std::string, double> _fixed_joints_map;

    Eigen::VectorXd _x, _u;
    talos_horizon::WBTrajectory _mpc_solution, _old_solution;
    Resampler::UniquePtr _resampler;

    // XBot::FlushMeMaybe::Ptr _flusher;

    XBot::ModelInterface::Ptr _model;
    XBot::RobotInterface::Ptr _robot;
    int _rate;

    bool _flag_id;
};

#endif // MPC_JOINT_HANDLER_H
