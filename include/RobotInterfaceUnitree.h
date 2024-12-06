#ifndef ROBOTINTERFACEUNITREE_H
#define ROBOTINTERFACEUNITREE_H

#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <algorithm>

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

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;

struct ImuState {
    std::array<float, 3> rpy = {};
    std::array<float, 3> omega = {};
};

struct MotorCommand {
    std::array<double, G1_NUM_MOTOR> q_target = {};
    std::array<double, G1_NUM_MOTOR> dq_target = {};
    std::array<double, G1_NUM_MOTOR> kp = {};
    std::array<double, G1_NUM_MOTOR> kd = {};
    std::array<double, G1_NUM_MOTOR> tau_ff = {};
};

struct MotorState {
    std::array<double, G1_NUM_MOTOR> q = {};
    std::array<double, G1_NUM_MOTOR> dq = {};
    std::array<double, G1_NUM_MOTOR> tau_est = {};
};

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

// Stiffness for all G1 Joints
std::array<double, G1_NUM_MOTOR> Kp{
    200, 200, 200, 500, 500, 500,     // legs
    200, 200, 200, 500, 500, 500,     // legs
    200, 150, 150,               // waist
    150, 150, 150, 150,  150, 150, 150,  // arms
    150, 150, 150, 150,  150, 150, 150   // arms
};

// Damping for all G1 Joints
std::array<double, G1_NUM_MOTOR> Kd{
    5, 5, 5, 10, 5, 5,     // legs
    5, 5, 5, 10, 5, 5,     // legs
    5, 5, 5,              // waist
    5, 5, 5, 5, 5, 5, 5,  // arms
    5, 5, 5, 5, 5, 5, 5   // arms
};

enum G1JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,  // NOTE INVALID for g1 23dof
  RightWristYaw = 28     // NOTE INVALID for g1 23dof
};

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &newData) {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = std::make_shared<T>(newData);
  }

  std::shared_ptr<const T> GetData() {
    std::shared_lock<std::shared_mutex> lock(mutex);
    return data ? data : nullptr;
  }

  void Clear() {
    std::unique_lock<std::shared_mutex> lock(mutex);
    data = nullptr;
  }

 private:
  std::shared_ptr<T> data;
  std::shared_mutex mutex;
};

class RobotInterfaceUnitree : public RobotInterface
{
    friend RobotInterface;

public:

    RobotInterfaceUnitree();

    virtual double getTime() const { return _time; }

    virtual bool isRunning() const { return true; }

protected:
    virtual bool post_init();
    virtual bool init_robot(const ConfigOptions& cfg);

    virtual bool sense_internal(bool sync_ref);
    virtual bool read_sensors() { return true; }
    virtual bool sense_hands() { return true; }

    virtual bool move_internal();
    virtual bool move_hands() { return true; }

private:
    void low_state_handler(const void* message);
    void low_command_writer();

    double _time;
    double _control_dt;  // [2ms]
    double _duration;    // [3 s]
    int _counter;
    Mode _mode_pr;
    uint8_t _mode_machine;

    DataBuffer<MotorState> _motor_state_buffer;
    DataBuffer<MotorCommand> _motor_command_buffer;
    DataBuffer<ImuState> _imu_state_buffer;

    ChannelPublisherPtr<LowCmd_> _lowcmd_publisher;
    ChannelSubscriberPtr<LowState_> _lowstate_subscriber;

    ThreadPtr command_writer_ptr_;

    Eigen::VectorXd _pos;
    Eigen::VectorXd _vel;
    Eigen::VectorXd _tau;
    Eigen::VectorXd _stiff;
    Eigen::VectorXd _damp;
};
}

#endif // ROBOTINTERFACEUNITREE_H
