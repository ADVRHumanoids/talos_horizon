#include "JointStatePublisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_state_publisher");

    ros::NodeHandle nh, nhpr("~");

    int rate;
    if (!nhpr.hasParam("rate"))
    {
        rate = 100;
    }
    else
    {
        nhpr.getParam("rate", rate);
    }

    std::cout << "running rate at " << rate << " Hz" << std::endl;

    XBot::JointStatePublisher js_pub(nh);

    ros::Rate r(rate);
    while (ros::ok())
    {
        js_pub.publish();
        ros::spinOnce();
    }
}
