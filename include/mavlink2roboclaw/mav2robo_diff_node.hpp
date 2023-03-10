// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>

#include "mav2robo_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "roboclaw/msg/motor_velocity_single_stamped.hpp"
#include "roboclaw/msg/motor_duty_single_stamped.hpp"
#include "mavros_msgs/msg/actuator_control.hpp"
#include "mavros_msgs/msg/actuator_output_status.hpp"

using namespace std::chrono_literals;
using namespace std;

namespace mav2robo
{
    class Mav2RoboDiff : public rclcpp::Node
    {
    public:
        Mav2RoboDiff(string name);
    private:
        // parameter variables
        RoboclawCmdType     mCmdType;
        uint8_t             mInputMixGroup;
        uint8_t             mInputThrottleChannel;
        uint8_t             mInputSteeringChannel;
        uint8_t             mLeftOutputIndex;
        uint8_t             mLeftOutputChannel;
        uint8_t             mRightOutputIndex;
        uint8_t             mRightOutputChannel;
        float               mSteeringGain;
        float               mThrottleGain;
        float               mSteeringOffset;
        float               mThrottleOffset;
        int32_t             mLeftOutGain;
        int32_t             mRightOutGain;
        int32_t             mLeftOutOffset;
        int32_t             mRightOutOffset;

        // subscriber
        rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr  mActSub;

        // publishers
        rclcpp::Publisher<roboclaw::msg::MotorVelocitySingleStamped>::SharedPtr    mLeftVelPub;
        rclcpp::Publisher<roboclaw::msg::MotorDutySingleStamped>::SharedPtr        mLeftDutyPub;
        rclcpp::Publisher<roboclaw::msg::MotorVelocitySingleStamped>::SharedPtr    mRightVelPub;
        rclcpp::Publisher<roboclaw::msg::MotorDutySingleStamped>::SharedPtr        mRightDutyPub;

        // callback
        void act_cb(const mavros_msgs::msg::ActuatorControl &msg);

        // publisher selector
        void pub(float left, float right);
    };
}