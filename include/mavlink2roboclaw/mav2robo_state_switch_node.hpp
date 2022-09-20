// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>
#include <vector>

#include "mav2robo_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "ssp_interfaces/srv/relay_command.hpp"
#include "mavros_msgs/msg/state.hpp"

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
    
        bool get_state() {return mOutputState; }
        uint8_t get_channel() { return mOutputChannel; }
        bool is_new_cmd() { return mNewCommandAvailable; }
        void clear_new_cmd() { mNewCommandAvailable = false; }

    private:
        // parameter variables
        bool            mConnectedEnabled;
        bool            mArmedEnabled;
        bool            mGuidedEnabled;
        bool            mManualInputEnabled;
        bool            mConnectedInverted;
        bool            mArmedInverted;
        bool            mGuidedInverted;
        bool            mManualInputInverted;
        bool            mIsOutputAnd;
        bool            mStringModeEnabled;
        bool            mStringModeInverted;
        vector<string>  mTriggerModes;
        bool            mSysStatusEnabled;
        bool            mSysStatusInverted;
        vector<uint8_t> mSysStatusTriggerValues;
        uint8_t         mOutputChannel;
        bool            mOutputInvert;

        // param handlers
        void declare_params();
        void fetch_params();

        // input handlers
        bool check_string_mode_triggered(string input);
        bool check_sys_state_triggered(uint8_t input);

        // subscriber
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mStateSub;

        // subscriber callback
        void act_cb(const mavros_msgs::msg::State &msg);

        // state variables
        bool    mModeTriggered;
        bool    mSysStatusTriggered;
        bool    mNewCommandAvailable;
        bool    mOutputState;

    };
} // namespace mav2robo