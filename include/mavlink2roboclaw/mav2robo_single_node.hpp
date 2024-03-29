// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>

#include "mav2robo_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "roboclaw/msg/motor_velocity_single_stamped.hpp"
#include "roboclaw/msg/motor_duty_single_stamped.hpp"
#include "roboclaw/msg/motor_position_single_stamped.hpp"
#include "mavros_msgs/msg/actuator_control.hpp"
#include "mavros_msgs/msg/actuator_output_status.hpp"
#include "mavros_msgs/msg/state.hpp"

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
        int8_t              mInputMixGroup;      // if this is less than 0, then use ActuatorOutputStatus inputs
        uint8_t             mInputCtrlChannel;
        uint8_t             mOutputIndex;
        uint8_t             mOutputChannel;
        int32_t             mGain;
        int32_t             mOffset;

        // state variable
        bool                mIsArmed;

        // subscriber
        rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr      mActSub;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr                mStateSub;
        rclcpp::Subscription<mavros_msgs::msg::ActuatorOutputStatus>::SharedPtr mActOutStatSub;

        // publishers
        rclcpp::Publisher<roboclaw::msg::MotorPositionSingleStamped>::SharedPtr mPosnPub;
        rclcpp::Publisher<roboclaw::msg::MotorVelocitySingleStamped>::SharedPtr mVelPub;
        rclcpp::Publisher<roboclaw::msg::MotorDutySingleStamped>::SharedPtr     mDutyPub;

        // callback
        void act_cb(const mavros_msgs::msg::ActuatorControl &msg);
        void act_output_cb(const mavros_msgs::msg::ActuatorOutputStatus &msg);
        void state_cb(const mavros_msgs::msg::State &msg_in);

        // publisher selector
        void pub(float val);
    };
} // namespace mav2robo