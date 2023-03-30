// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>

#include "mav2robo_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ssp_interfaces/srv/relay_command.hpp"
#include "mavros_msgs/msg/actuator_control.hpp"
#include "mavros_msgs/msg/actuator_output_status.hpp"

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
        bool is_new_cmd() { return mNewCommand; }
        void clear_new_cmd() { mNewCommand = false; }

    private:
        // parameter variables
        uint8_t             mInputMixGroup;      // if this is less than 0, then use ActuatorOutputStatus inputs
        uint8_t             mInputCtrlChannel;
        float               mThreshold;
        uint8_t             mOutputChannel;
        bool                mOutputInvert;

        // subscriber
        rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr      mActSub;
        rclcpp::Subscription<mavros_msgs::msg::ActuatorOutputStatus>::SharedPtr mActOutStatSub;

        // subscriber callback
        void act_cb(const mavros_msgs::msg::ActuatorControl &msg);
        void act_out_stat_cb(const mavros_msgs::msg::ActuatorOutputStatus &msg);

        // state variables
        float               mInputValue;
        bool                mNewCommand;
        bool                mLastCommand;

    };
} // namespace mav2robo