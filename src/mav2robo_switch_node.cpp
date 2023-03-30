// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#include "mavlink2roboclaw/mav2robo_switch_node.hpp"
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;

namespace mav2robo
{
    Mav2RoboSwitch::Mav2RoboSwitch(string name) : Node (name)
    {
        // declare params
        this->declare_parameter<uint8_t>("input_mix_group", 0);
        this->declare_parameter<uint8_t>("input_ctrl_chan", 0);
        this->declare_parameter<uint8_t>("output_channel", 1);
        this->declare_parameter<uint8_t>("switch_threshold", 0.0);
        this->declare_parameter<bool>("output_invert", false);

        // fetch params
        this->get_parameter("input_mix_group", mInputMixGroup);
        this->get_parameter("input_ctrl_chan", mInputCtrlChannel);
        this->get_parameter("output_channel", mOutputChannel);
        this->get_parameter("switch_threshold", mThreshold);
        this->get_parameter("output_invert", mOutputInvert);
        if (mInputCtrlChannel > 7) { throw std::runtime_error("Input channel must be between 0 and 7, inclusive"); }
        if ((mOutputChannel > 8) or (mOutputChannel < 1)) { throw std::runtime_error("Output channel must be between 1 and 8, inclusive"); }

        // create subscriber
        mActSub = this->create_subscription<mavros_msgs::msg::ActuatorControl>(
            "~/act_cmd", 10, bind(&Mav2RoboSwitch::act_cb, this, _1));
        mActOutStatSub = this->create_subscription<mavros_msgs::msg::ActuatorActuatorOutputStatusControl>(
            "~/act_cmd", 10, bind(&Mav2RoboSwitch::act_out_stat_cb, this, _1));

        // create client
        mRelayClient = this->create_client<ssp_interfaces::srv::RelayCommand>("~/relay_client");
    }

    void Mav2RoboSwitch::act_cb(const mavros_msgs::msg::ActuatorControl &msg)
    {
        if (msg.group_mix == mInputMixGroup) 
        { 
            mInputValue = msg.controls[mInputCtrlChannel];
            mNewCommand = true; 
        }
    }

    void Mav2RoboSwitch::act_out_stat_cb(const mavros_msgs::msg::ActuatorOutputStatus &msg)
    {
        if (msg.group_mix < 0) 
        { 
            mInputValue = msg.actuator[mInputCtrlChannel];
            mNewCommand = true; 
        }
    }

    bool Mav2RoboSwitch::get_state()
    {
        if (mInputValue > mThreshold)
        {
            return (!mOutputInvert);
        }
        return mOutputInvert;
    }

} // namespace mav2robo

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mav2robo::Mav2RoboSwitch>("mav2roboclaw_switch");
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
    }
    rclcpp::shutdown();

    return 0;
}