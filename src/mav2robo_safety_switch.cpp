// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#include "mavlink2roboclaw/mav2robo_safety_switch_node.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <bitset>
#include <thread>

using std::placeholders::_1;

namespace mav2robo
{
    Mav2RoboSafetySwitch::Mav2RoboSafetySwitch(string name) : Node (name)
    {
        declare_params();
        fetch_params();

        // create subscriber
        mStateSub = this->create_subscription<mavros_msgs::msg::State>(
            "~/state", 10, bind(&Mav2RoboSafetySwitch::state_cb, this, _1));
        mTriggerSub = this->create_subscription<ssp_interfaces::msg::DigitalInput>(
            "~/trigger", 10, bind(&Mav2RoboSafetySwitch::trigger_cb, this, _1));

        // create client
        mRelayClient = this->create_client<ssp_interfaces::srv::RelayCommand>("~/relay_client");

        // create timer
        mResponseTimer = this->create_wall_timer(100ms, 
            bind(&Mav2RoboSafetySwitch::timer_cb, this));

        // state variable initialization
        mResetEnable = false;
        mIsLatched = false;
    }

    void Mav2RoboSafetySwitch::declare_params()
    {
        this->declare_parameter<bool>("reset.connect.enabled", false);
        this->declare_parameter<bool>("reset.armed.enabled", false);
        this->declare_parameter<bool>("reset.guided.enabled", false);
        this->declare_parameter<bool>("reset.manual_input.enabled", false);
        this->declare_parameter<bool>("reset.string_mode.enabled", false);
        this->declare_parameter<bool>("reset.system_status.enabled", false);
        this->declare_parameter<bool>("reset.connect.inverted", false);
        this->declare_parameter<bool>("reset.armed.inverted", false);
        this->declare_parameter<bool>("reset.guided.inverted", false);
        this->declare_parameter<bool>("reset.manual.input_inverted", false);
        this->declare_parameter<bool>("reset.string_mode.inverted", false);
        this->declare_parameter<bool>("reset.system_status.inverted", false);
        this->declare_parameter<bool>("reset.output.and", false);
        this->declare_parameter<vector<string>>("reset.string_mode.triggers", vector<string>{});
        this->declare_parameter<vector<uint8_t>>("reset.system_status.triggers", vector<uint8_t>{});
        this->declare_parameter<uint8_t>("output.channel", 1);
        this->declare_parameter<bool>("output.triggered", false);
        this->declare_parameter<bool>("input.triggered", false);
    }

    void Mav2RoboSafetySwitch::fetch_params()
    {
        this->get_parameter("reset.connect.enabled", mConnectedEnabled);
        this->get_parameter("reset.armed.enabled", mArmedEnabled);
        this->get_parameter("reset.guided.enabled", mGuidedEnabled);
        this->get_parameter("reset.manual_input.enabled", mManualInputEnabled);
        this->get_parameter("reset.connect.inverted", mConnectedInverted);
        this->get_parameter("reset.armed.inverted", mArmedInverted);
        this->get_parameter("reset.guided.inverted", mGuidedInverted);
        this->get_parameter("reset.manual_input.input_inverted", mManualInputInverted);
        this->get_parameter("reset.output.and", mIsOutputAnd);
        this->get_parameter("reset.string_mode.enabled", mStringModeEnabled);
        this->get_parameter("reset.string_mode.inverted", mStringModeInverted);
        this->get_parameter("reset.string_mode.triggers", mTriggerModes);
        this->get_parameter("reset.system_status.enabled", mSysStatusEnabled);
        this->get_parameter("reset.system_status.inverted", mSysStatusInverted);
        this->get_parameter("reset.system_status.triggers", mSysStatusTriggerValues);
        this->get_parameter("output.channel", mOutputChannel);
        this->get_parameter("output.triggered", mTriggeredOutput);
        this->get_parameter("input.triggered", mTriggerActiveState);
        if ((mOutputChannel > 8) or (mOutputChannel < 1)) { throw std::runtime_error("Output channel must be between 1 and 8, inclusive"); }
    }

    void Mav2RoboSafetySwitch::state_cb(const mavros_msgs::msg::State &msg)
    {
        bitset<6> reset_states;
        bool reset = false;
        // For enabled trigger modes, this logic applies the incoming state. 
        // Otherwise, it applies the value of mIsOutputAnd, which makes sure both OR and AND logic work correctly
        reset_states[0] = mStringModeEnabled ? (check_string_mode_triggered(msg.mode)) : mIsOutputAnd;
        reset_states[1] = mSysStatusEnabled ? (check_sys_state_triggered(msg.system_status)) : mIsOutputAnd;
        reset_states[2] = mConnectedEnabled ? (msg.connected xor mConnectedInverted) : mIsOutputAnd;
        reset_states[3] = mArmedEnabled ? (msg.armed xor mArmedInverted) : mIsOutputAnd;
        reset_states[4] = mGuidedEnabled ? (msg.guided xor mGuidedInverted) : mIsOutputAnd;
        reset_states[5] = mManualInputEnabled ? (msg.manual_input xor mManualInputInverted) : mIsOutputAnd;
        if (mIsOutputAnd) { reset = reset_states.all(); }
        else { reset = reset_states.any(); }
        if (!reset) { mResetEnable = true; }
        if (reset and mIsLatched and mResetEnable) { 
            RCLCPP_INFO(this->get_logger(), "Resetting safety switch");
            mNewCommandAvailable = true; 
            mIsLatched = false;
            mOutputState = !mTriggeredOutput;
        }
        
    }

    void Mav2RoboSafetySwitch::trigger_cb(const ssp_interfaces::msg::DigitalInput &msg)
    {
        if (msg.state == mTriggerActiveState) { 
            mIsLatched = true; 
            mResetEnable = false;
        }
    }
    
    bool Mav2RoboSafetySwitch::check_string_mode_triggered(string input)
    {
        for (auto &a : mTriggerModes) { if(a == input) { return true; } }
        return false;
    }

    bool Mav2RoboSafetySwitch::check_sys_state_triggered(uint8_t input)
    {
        for (auto &a : mSysStatusTriggerValues) { if(a == input) { return true; } }
        return false;
    }

    void Mav2RoboSafetySwitch::timer_cb() 
    {
        if (mIsLatched)
        {
            mNewCommandAvailable = true;
            mOutputState = mTriggeredOutput;
        }
    }

} // namespace mav2robo

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mav2robo::Mav2RoboSafetySwitch>("mav2roboclaw_state_switch");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->is_new_cmd() and node->mRelayClient->wait_for_service(100ms))
        {
            RCLCPP_INFO(node->get_logger(), "Writing relay command %d to relay  %d", 
                node->get_state(), node->get_channel());
            auto request = std::make_shared<ssp_interfaces::srv::RelayCommand::Request>();
            request->channel = node->get_channel();
            request->state = node->get_state();
            auto result = node->mRelayClient->async_send_request(request);
            if ((rclcpp::spin_until_future_complete(node, result) 
                == rclcpp::FutureReturnCode::SUCCESS)
                and (result.get()->state == request->state))
            {
                node->clear_new_cmd();
                RCLCPP_INFO(node->get_logger(), "Successfully set relay %d to state %d", request->channel, request->state);
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "Failed to set relay %d to state %d", request->channel, request->state);
            }
        }
        // This is a stupid ugly hack to keep this thread from slamming the CPU it's running on -- PN 9/26/22
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    rclcpp::shutdown();

    return 0;
}