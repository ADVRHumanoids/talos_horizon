#ifndef JOINTSTATEPUBLISHER_H
#define JOINTSTATEPUBLISHER_H

#include <xbot_msgs/JointState.h>
#include <XBotInterface/RobotInterface.h>

#include <ros/ros.h>

//#include "RobotInterfaceUnitree.h"

namespace XBot {


class JointStatePublisher {
public:
    JointStatePublisher(ros::NodeHandle nh);

    void publish();
private:
    void init_robot();

    ros::NodeHandle _nh, _nhpr;
    ros::Publisher _js_pub;

    XBot::RobotInterface::Ptr _robot;

    Eigen::VectorXd _q;
    Eigen::VectorXd _q_ref;
    Eigen::VectorXd _qdot;
    Eigen::VectorXd _qdot_ref;
    Eigen::VectorXd _stiff;
    Eigen::VectorXd _damp;
    Eigen::VectorXd _tau_ff;

    std::vector<std::string> _joint_names;


};
}

#endif // JOINTSTATEPUBLISHER_H
