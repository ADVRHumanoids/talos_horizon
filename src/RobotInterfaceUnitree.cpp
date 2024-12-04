#include "RobotInterfaceUnitree.h"

#include <RobotInterfaceROS/RobotInterfaceROS.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <XBotInterface/RtLog.hpp>

#include <XBotInterface/SoLib.h>
REGISTER_SO_LIB_(XBot::RobotInterfaceUnitree, XBot::RobotInterface);

XBot::RobotInterfaceUnitree::RobotInterfaceUnitree()
{
    _pos.resize(G1_NUM_MOTOR);
    _vel.resize(G1_NUM_MOTOR);
    _stiff.resize(G1_NUM_MOTOR);
    _damp.resize(G1_NUM_MOTOR);
    _tau.resize(G1_NUM_MOTOR);
}

bool XBot::RobotInterfaceUnitree::post_init()
{
    /* Initialize all RX and TX fields */
    _motor_state_buffer.Clear();
    while(!XBot::RobotInterfaceUnitree::sense_internal(true))
    {
        sleep(1);
        std::cout << "Waiting for first joint state message.. \n";
    }
    return true;
}

bool XBot::RobotInterfaceUnitree::init_robot(const ConfigOptions &cfg)
{
    // Print out that RobotInterfaceUnitree is begin initialized!
    Logger::info("Constructing Unitree implementation of RobotInterface.. \n");

    ChannelFactory::Instance()->Init(1, "lo");

    // create publisher
    _lowcmd_publisher.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    _lowcmd_publisher->InitChannel();
    // create subscriber
    _lowstate_subscriber.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    _lowstate_subscriber->InitChannel(std::bind(&RobotInterfaceUnitree::low_state_handler, this, std::placeholders::_1), 1);
    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, 2000, &RobotInterfaceUnitree::low_command_writer, this);
//    control_thread_ptr_ = CreateRecurrentThreadEx("control", UT_CPU_ID_NONE, 2000, &RobotInterfaceUnitree::control, this);

    Logger::success("RobotInterfacUnitree init completed! \n");

    sleep(1.);

    return true;
}

void XBot::RobotInterfaceUnitree::low_state_handler(const void *message)
{
//    std::cout << "RECEIVED" << std::endl;
    LowState_ low_state = *(const LowState_ *)message;

    // get motor state
    MotorState ms_tmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        ms_tmp.q.at(i) = low_state.motor_state()[i].q();
        ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
        if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
            std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
    }
    _motor_state_buffer.SetData(ms_tmp);

    // get imu state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    _imu_state_buffer.SetData(imu_tmp);
}

bool XBot::RobotInterfaceUnitree::sense_internal(bool sync_ref)
{
    std::shared_ptr<const MotorState> ms_tmp = _motor_state_buffer.GetData();

    if (!ms_tmp)
    {
        return false;
    }

    for (int i = 0; i < G1_NUM_MOTOR; i++)
    {
        _pos(i) = ms_tmp->q.at(i);
        _stiff(i) = Kp.at(i);
        _damp(i) = Kd.at(i);
    }

    if(sync_ref)
    {
        setPositionReference(_pos);
        setStiffness(_stiff);
        setDamping(_damp);
    }

    return true;
}

void XBot::RobotInterfaceUnitree::low_command_writer()
{
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(_mode_pr);
    dds_low_command.mode_machine() = _mode_machine;

    const std::shared_ptr<const MotorCommand> mc = _motor_command_buffer.GetData();
    if (mc)
    {
        for (size_t i = 0; i < G1_NUM_MOTOR; i++)
        {
            dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
            dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
            dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
            dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
            dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
            dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
        }

    _lowcmd_publisher->Write(dds_low_command);
    }
}

bool XBot::RobotInterfaceUnitree::move_internal()
{
    MotorCommand motor_command_tmp;

    getPositionReference(_pos);
    getVelocityReference(_vel);
    getEffortReference(_tau);
    getStiffness(_stiff);
    getDamping(_tau);

    for (int i = 0; i < G1_NUM_MOTOR; i++)
    {
        motor_command_tmp.q_target.at(i) = _pos(i);
        motor_command_tmp.dq_target.at(i) = _vel(i);
        motor_command_tmp.tau_ff.at(i) = _tau(i);
        motor_command_tmp.kp.at(i) = _stiff(i);
        motor_command_tmp.kd.at(i) = _damp(i);
    }
    _motor_command_buffer.SetData(motor_command_tmp);

    return true;
}

void XBot::RobotInterfaceUnitree::control()
{
//    MotorCommand motor_command_tmp;
//    const std::shared_ptr<const MotorState> ms = _motor_state_buffer.GetData();

//    for (int i = 0; i < G1_NUM_MOTOR; ++i)
//    {
//        motor_command_tmp.tau_ff.at(i) = 0.0;
//        motor_command_tmp.q_target.at(i) = 0.0;
//        motor_command_tmp.dq_target.at(i) = 0.0;
//        motor_command_tmp.kp.at(i) = Kp[i];
//        motor_command_tmp.kd.at(i) = Kd[i];
//    }

//    if (ms)
//    {
//        _time += _control_dt;
//        if (_time < _duration)
//        {
//            // [Stage 1]: set robot to zero posture
//            for (int i = 0; i < G1_NUM_MOTOR; ++i)
//            {
//                double ratio = std::clamp(_time / _duration, 0.0, 1.0);
//                motor_command_tmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
//            }
//        }
//        else if (_time < _duration * 2)
//        {
//            // [Stage 2]: swing ankle using PR mode
//            _mode_pr = Mode::PR;
//            double max_P = M_PI * 30.0 / 180.0;
//            double max_R = M_PI * 10.0 / 180.0;
//            double t = _time - _duration;
//            double L_P_des = max_P * std::sin(2.0 * M_PI * t);
//            double L_R_des = max_R * std::sin(2.0 * M_PI * t);
//            double R_P_des = max_P * std::sin(2.0 * M_PI * t);
//            double R_R_des = -max_R * std::sin(2.0 * M_PI * t);

//            motor_command_tmp.q_target.at(LeftAnklePitch) = L_P_des;
//            motor_command_tmp.q_target.at(LeftAnkleRoll) = L_R_des;
//            motor_command_tmp.q_target.at(RightAnklePitch) = R_P_des;
//            motor_command_tmp.q_target.at(RightAnkleRoll) = R_R_des;
//        }
//        else
//        {
//            // [Stage 3]: swing ankle using AB mode
//            _mode_pr = Mode::AB;
//            double max_A = M_PI * 30.0 / 180.0;
//            double max_B = M_PI * 10.0 / 180.0;
//            double t = _time - _duration * 2;
//            double L_A_des = +max_A * std::sin(M_PI * t);
//            double L_B_des = +max_B * std::sin(M_PI * t + M_PI);
//            double R_A_des = -max_A * std::sin(M_PI * t);
//            double R_B_des = -max_B * std::sin(M_PI * t + M_PI);

//            motor_command_tmp.q_target.at(LeftAnkleA) = L_A_des;
//            motor_command_tmp.q_target.at(LeftAnkleB) = L_B_des;
//            motor_command_tmp.q_target.at(RightAnkleA) = R_A_des;
//            motor_command_tmp.q_target.at(RightAnkleB) = R_B_des;
//        }

//        _motor_command_buffer.SetData(motor_command_tmp);
//    }
}

