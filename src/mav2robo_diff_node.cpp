// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#include "mavlink2roboclaw/mav2robo_diff_node.hpp"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;

namespace mav2robo
{

    Mav2RoboDiff::Mav2RoboDiff(string name) : Node (name)
    {
        // declare params
        this->declare_parameter<std::string>("command_type",        "duty");
        this->declare_parameter<uint8_t>("input.mix_group",         0);
        this->declare_parameter<uint8_t>("input.throttle.channel",  0);
        this->declare_parameter<uint8_t>("input.steering.channel",  1);
        this->declare_parameter<float>("mix.steering.gain",         1.0f);
        this->declare_parameter<float>("mix.steering.offset",       0.0f);
        this->declare_parameter<float>("mix.throttle.gain",         1.0f);
        this->declare_parameter<float>("mix.throttle.offset",       0.0f);
        this->declare_parameter<uint8_t>("output.left.index",       0);
        this->declare_parameter<uint8_t>("output.left.channel",     1);
        this->declare_parameter<uint8_t>("output.right.index",      0);
        this->declare_parameter<uint8_t>("output.right.channel",    2);
        this->declare_parameter<int32_t>("output.left.gain",        1024);
        this->declare_parameter<int32_t>("output.left.offset",      0);
        this->declare_parameter<int32_t>("output.right.gain",       1024);
        this->declare_parameter<int32_t>("output.right.offset",     0);

        // fetch params
        std::string tmp;
        this->get_parameter("command_type", tmp);
        mav2robo::tolower_str(tmp);
        if (tmp == "velocity") { mCmdType = mav2robo::RoboclawCmdType::Velocity; }
        else if (tmp == "duty") { mCmdType = mav2robo::RoboclawCmdType::Duty; }
        else { throw std::runtime_error("command_type must be either \"velocity\" or \"duty\""); }
        std::string sub, pub;
        this->get_parameter("input.mix_group",          mInputMixGroup);
        this->get_parameter("input.throttle.channel",   mInputThrottleChannel);
        this->get_parameter("input.steering.channel",   mInputSteeringChannel);
        this->get_parameter("mix.steering.gain",        mSteeringGain);
        this->get_parameter("mix.steering.offset",      mThrottleGain);
        this->get_parameter("mix.throttle.gain",        mSteeringOffset);
        this->get_parameter("mix.throttle.offset",      mThrottleOffset);
        this->get_parameter("output.left.index",        mLeftOutputIndex);
        this->get_parameter("output.left.channel",      mLeftOutputChannel);
        this->get_parameter("output.right.index",       mRightOutputIndex);
        this->get_parameter("output.right.channel",     mRightOutputChannel);
        this->get_parameter("output.left.gain",         mLeftOutGain);
        this->get_parameter("output.left.offset",       mRightOutGain);
        this->get_parameter("output.right.gain",        mLeftOutOffset);
        this->get_parameter("output.right.offset",      mRightOutOffset);
        if (mInputSteeringChannel > 7) { throw std::runtime_error("Steering input channel must be between 0 and 7, inclusive"); }
        if (mInputThrottleChannel > 7) { throw std::runtime_error("Throttle input channel must be between 0 and 7, inclusive"); }

        // create subscriber
        mActSub = this->create_subscription<mavros_msgs::msg::ActuatorControl>(
            "~/act_cmd", 10, bind(&Mav2RoboDiff::act_cb, this, _1));

        // create publisher
        switch(mCmdType)
        {
            case mav2robo::RoboclawCmdType::Velocity:
                mLeftVelPub = this->create_publisher<roboclaw::msg::MotorVelocitySingleStamped>("~/left/vel_cmd", 10);
                mRightVelPub = this->create_publisher<roboclaw::msg::MotorVelocitySingleStamped>("~/right/vel_cmd", 10);
                break;
            case mav2robo::RoboclawCmdType::Duty:
                mLeftDutyPub = this->create_publisher<roboclaw::msg::MotorDutySingleStamped>("~/left/duty_cmd", 10);
                mRightDutyPub = this->create_publisher<roboclaw::msg::MotorDutySingleStamped>("~/right/duty_cmd", 10);
                break;
            default:
                mLeftDutyPub = this->create_publisher<roboclaw::msg::MotorDutySingleStamped>("~/left/duty_cmd", 10);
                mRightDutyPub = this->create_publisher<roboclaw::msg::MotorDutySingleStamped>("~/right/duty_cmd", 10);
                break;
        }
    }

    void Mav2RoboDiff::act_cb(const mavros_msgs::msg::ActuatorControl &msg)
    {
        if (msg.group_mix == mInputMixGroup)
        {
            float left = (((msg.controls[mInputThrottleChannel] * mThrottleGain) + mThrottleOffset)
                            - ((msg.controls[mInputSteeringChannel] * mSteeringGain) + mSteeringOffset));
            float right = (((msg.controls[mInputThrottleChannel] * mThrottleGain) + mThrottleOffset)
                            + ((msg.controls[mInputSteeringChannel] * mSteeringGain) + mSteeringOffset));
            pub(left, right);
        }
    }

    void Mav2RoboDiff::pub(float left, float right)
    {
        int32_t left_out = (fbound(left, 1.0f, -1.0f) * mLeftOutGain) + mLeftOutOffset;
        int32_t right_out = (fbound(right, 1.0f, -1.0f) * mRightOutGain) + mRightOutOffset;
    
        switch(mCmdType)
        {
            case mav2robo::RoboclawCmdType::Velocity:
            {
                auto msg_left = roboclaw::msg::MotorVelocitySingleStamped();
                auto msg_right = roboclaw::msg::MotorVelocitySingleStamped();
                msg_left.header.stamp = this->get_clock()->now();
                msg_right.header.stamp = this->get_clock()->now();
                msg_left.index = mLeftOutputIndex;
                msg_left.channel = mLeftOutputChannel;
                msg_left.mot_vel_sps = left_out;
                msg_right.index = mRightOutputIndex;
                msg_right.channel = mRightOutputChannel;
                msg_right.mot_vel_sps = right_out;
                mLeftVelPub->publish(msg_left);
                mRightVelPub->publish(msg_right);
                break;
            }
            case mav2robo::RoboclawCmdType::Duty:
            default:
            {
                auto msg_left = roboclaw::msg::MotorDutySingleStamped();
                auto msg_right = roboclaw::msg::MotorDutySingleStamped();
                msg_left.header.stamp = this->get_clock()->now();
                msg_right.header.stamp = this->get_clock()->now();
                msg_left.index = mLeftOutputIndex;
                msg_left.channel = mLeftOutputChannel;
                msg_left.mot_duty = left_out;
                msg_right.index = mRightOutputIndex;
                msg_right.channel = mRightOutputChannel;
                msg_right.mot_duty = right_out;
                mLeftDutyPub->publish(msg_left);  
                mRightDutyPub->publish(msg_right);                
                break;
            }
        }
    }
} // namespace mav2robo

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mav2robo::Mav2RoboDiff>("mav2roboclaw_diff"));   rclcpp::shutdown();

    return 0;
}