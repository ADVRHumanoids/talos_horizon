#ifndef ROBOTINTERFACEUNITREE_H
#define ROBOTINTERFACEUNITREE_H

#include <XBotInterface/RobotInterface.h>

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

namespace XBot {

class RobotInterfaceUnitree : public RobotInterface
{

    friend RobotInterface;

public:

    RobotInterfaceUnitree();
};
}

#endif // ROBOTINTERFACEUNITREE_H
