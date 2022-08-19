// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>

#include "mav2robo_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ssp_interfaces/srv/relay_command.hpp"
#include "mavros_msgs/msg/actuator_control.hpp"

using namespace std::chrono_literals;
using namespace std;

namespace mav2robo
{
    class Mav2RoboSwitch : public rclcpp::Node
    {
    public:
        Mav2RoboSwitch(string name);
        // service client
        rclcpp::Client<ssp_interfaces::srv::RelayCommand>::SharedPtr mRelayClient;
    
        bool get_state();
        uint8_t get_channel() { return mOutputChannel; }

    private:
        // parameter variables
        uint8_t             mInputMixGroup;
        uint8_t             mInputCtrlChannel;
        float               mThreshold;
        uint8_t             mOutputChannel;
        bool                mOutputInvert;

        // subscriber
        rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr   mActSub;

        // subscriber callback
        void act_cb(const mavros_msgs::msg::ActuatorControl &msg);

        // state variables
        float               mInputValue;
        bool                mNewCommand;
        bool                mLastCommand;

    };
} // namespace mav2robo