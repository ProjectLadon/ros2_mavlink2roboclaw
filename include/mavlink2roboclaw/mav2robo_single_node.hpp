// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>

#include "mav2robo_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "roboclaw/msg/motor_velocity_single.hpp"
#include "roboclaw/msg/motor_duty_single.hpp"
#include "roboclaw/msg/motor_position_single.hpp"
#include "mavros_msgs/msg/actuator_control.hpp"
#include "mavros_msgs/msg/actuator_output_status.hpp"

using namespace std::chrono_literals;
using namespace std;

namespace mav2robo
{
    class Mav2RoboSingle : public rclcpp::Node
    {
    public:
        Mav2RoboSingle(string name);
    private:
        // parameter variables
        RoboclawCmdType     mCmdType;
        uint8_t             mInputMixGroup;
        uint8_t             mInputCtrlChannel;
        uint8_t             mOutputIndex;
        uint8_t             mOutputChannel;
        int32_t             mGain;
        int32_t             mOffset;

        // subscriber
        rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr  mActSub;

        // publishers
        rclcpp::Publisher<roboclaw::msg::MotorPositionSingle>::SharedPtr    mPosnPub;
        rclcpp::Publisher<roboclaw::msg::MotorVelocitySingle>::SharedPtr    mVelPub;
        rclcpp::Publisher<roboclaw::msg::MotorDutySingle>::SharedPtr        mDutyPub;

        // callback
        void act_cb(const mavros_msgs::msg::ActuatorControl &msg);

        // publisher selector
        void pub(float val);
    };
} // namespace mav2robo