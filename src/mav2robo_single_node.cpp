// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#include "mavlink2roboclaw/mav2robo_single_node.hpp"
#include <chrono>
#include <functional>
#include <memory>

namespace mav2robo
{
    Mav2RoboSingle::Mav2RoboSingle(string name) : Node (name)
    {
        // declare params
        this->declare_parameter<std::string>("command_type", "position");
        this->declare_parameter<int8_t>("input_mix_group", 0);
        this->declare_parameter<uint8_t>("input_ctrl_chan", 0);
        this->declare_parameter<uint8_t>("output_index", 0);
        this->declare_parameter<uint8_t>("output_channel", 1);
        this->declare_parameter<int32_t>("gain", 1024);
        this->declare_parameter<int32_t>("offset", 0);

        // fetch params
        std::string tmp;
        this->get_parameter("command_type", tmp);
        mav2robo::tolower_str(tmp);
        if (tmp == "position") { mCmdType = mav2robo::RoboclawCmdType::Position; }
        else if (tmp == "velocity") { mCmdType = mav2robo::RoboclawCmdType::Velocity; }
        else if (tmp == "duty") { mCmdType = mav2robo::RoboclawCmdType::Duty; }
        else { throw std::runtime_error("command_type must be either \"position\", \"velocity\", or \"duty\""); }
        std::string sub, pub;
        this->get_parameter("input_mix_group", mInputMixGroup);
        this->get_parameter("input_ctrl_chan", mInputCtrlChannel);
        this->get_parameter("output_index", mOutputIndex);
        this->get_parameter("output_channel", mOutputChannel);
        this->get_parameter("gain", mGain);
        this->get_parameter("offset", mOffset);
        if (mInputCtrlChannel > 7) { throw std::runtime_error("Input channel must be between 0 and 7, inclusive"); }

        // create subscriber
        auto sensor_qos     = rclcpp::SensorDataQoS();
        mActSub = this->create_subscription<mavros_msgs::msg::ActuatorControl>(
            "~/act_cmd", sensor_qos, bind(&Mav2RoboSingle::act_cb, this, std::placeholders::_1));
        mActOutStatSub = this->create_subscription<mavros_msgs::msg::ActuatorOutputStatus>(
            "~/act_cmd", sensor_qos, bind(&Mav2RoboSingle::act_output_cb, this, std::placeholders::_1));
        mStateSub = this->create_subscription<mavros_msgs::msg::State>(
            "~/state_in", sensor_qos, bind(&Mav2RoboSingle::state_cb, this, std::placeholders::_1));

        // create publishers
        mDutyPub = this->create_publisher<roboclaw::msg::MotorDutySingleStamped>("~/duty_cmd", 10);
        switch(mCmdType)
        {
            case mav2robo::RoboclawCmdType::Velocity:
                mVelPub = this->create_publisher<roboclaw::msg::MotorVelocitySingleStamped>("~/vel_cmd", 10);
                break;
            case mav2robo::RoboclawCmdType::Duty:
                break;
            case mav2robo::RoboclawCmdType::Position:
            default:
                mPosnPub = this->create_publisher<roboclaw::msg::MotorPositionSingleStamped>("~/posn_cmd", 10);
                break;
        }
    }

    void Mav2RoboSingle::act_cb(const mavros_msgs::msg::ActuatorControl &msg)
    {
        if (msg.group_mix == mInputMixGroup) { pub(msg.controls[mInputCtrlChannel]); }
    }

    void Mav2RoboSingle::act_output_cb(const mavros_msgs::msg::ActuatorOutputStatus &msg)
    {
        if (mInputMixGroup < 0) { pub(msg.actuator[mInputCtrlChannel]); }
    }

    void Mav2RoboSingle::state_cb(const mavros_msgs::msg::State &msg_in)
    {
        if (mIsArmed xor msg_in.armed)
        {
            auto msg_out = roboclaw::msg::MotorDutySingleStamped();
            msg_out.header.stamp = this->get_clock()->now();
            msg_out.index = mOutputIndex;
            msg_out.channel = mOutputChannel;
            msg_out.mot_duty = 0;
            mDutyPub->publish(msg_out);    
            mIsArmed = msg_in.armed;          
        }
    }

    void Mav2RoboSingle::pub(float val)
    {
        int32_t out = (val * mGain) + mOffset;
        switch(mCmdType)
        {
            case mav2robo::RoboclawCmdType::Velocity:
            {
                auto msg = roboclaw::msg::MotorVelocitySingleStamped();
                msg.header.stamp = this->get_clock()->now();
                msg.index = mOutputIndex;
                msg.channel = mOutputChannel;
                msg.mot_vel_sps = out;
                mVelPub->publish(msg);
                break;
            }
            case mav2robo::RoboclawCmdType::Duty:
            {
                auto msg = roboclaw::msg::MotorDutySingleStamped();
                msg.header.stamp = this->get_clock()->now();
                msg.index = mOutputIndex;
                msg.channel = mOutputChannel;
                msg.mot_duty = out;
                mDutyPub->publish(msg);                
                break;
            }
            case mav2robo::RoboclawCmdType::Position:
            default:
            {
                auto msg = roboclaw::msg::MotorPositionSingleStamped();
                msg.header.stamp = this->get_clock()->now();
                msg.index = mOutputIndex;
                msg.channel = mOutputChannel;
                msg.mot_pos_steps = out;
                mPosnPub->publish(msg);                
                break;
            }
        }
    }

} // namespace mav2robo

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mav2robo::Mav2RoboSingle>("mav2roboclaw_single"));
    rclcpp::shutdown();

    return 0;
}