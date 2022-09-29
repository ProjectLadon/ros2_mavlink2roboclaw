// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>


#include "mavlink2roboclaw/mav2robo_common.hpp"
#include "mavlink2roboclaw/mav2robo_aux_propulsion_node.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std;
using std::placeholders::_1;

namespace mav2robo
{
    Mav2RoboAuxProp::Mav2RoboAuxProp(string name) : Node (name)
    {
        // initialize state variables
        mCurrState              = aux_propulsion_state_t::BEGIN;
        mLastState              = aux_propulsion_state_t::BEGIN;
        mHornFlag               = false;
        mHornState              = false;
        mExtendCmdFlag          = false;
        mStateStartTime         = steady_clock::now();
        mThrottleCmd            = 0.0;
        mSteeringCmd            = 0.0;
        mRetractLeftAmps        = 0.0;
        mRetractRightAmps       = 0.0;
        mRetractLeftComplete    = false;
        mRetractRightComplete   = false;

        mParamSub           = std::make_shared<rclcpp::ParameterEventHandler>(this);
        declare_params();
        fetch_params();

        // create subscribers
        auto sensor_qos     = rclcpp::SensorDataQoS();
        mActCtrlSub         = this->create_subscription<mavros_msgs::msg::ActuatorControl>(
            "~/actuator_control", sensor_qos, 
            bind(&Mav2RoboAuxProp::act_ctrl_cb, this, _1));
        mActOutStatSub      = this->create_subscription<mavros_msgs::msg::ActuatorOutputStatus>(
            "~/actuator_output_status", sensor_qos, 
            bind(&Mav2RoboAuxProp::act_out_stat_cb, this, _1));
        mMotorCurrentSub    = this->create_subscription<roboclaw::msg::MotorVoltsAmps>(
            "~/retract_current", sensor_qos, 
            bind(&Mav2RoboAuxProp::motor_current_cb, this, _1));
        
        // create publishers
        mRightMotorPub      = this->create_publisher<roboclaw::msg::MotorDutySingle>("~/right/propulsion", sensor_qos);
        mLeftMotorPub       = this->create_publisher<roboclaw::msg::MotorDutySingle>("~/left/propulsion", sensor_qos);
        mRightRetractPub    = this->create_publisher<roboclaw::msg::MotorDutySingle>("~/right/retract", sensor_qos);    
        mLeftRetractPub     = this->create_publisher<roboclaw::msg::MotorDutySingle>("~/left/retract", sensor_qos);

        // create client
        mHornClient         = this->create_client<ssp_interfaces::srv::RelayCommand>("~/horn");

        // create timer callback 
        mStateTimer         = this->create_wall_timer(mStepTimeMillis, 
            bind(&Mav2RoboAuxProp::state_timer_cb, this));

    }

    bool Mav2RoboAuxProp::horn_prepare(
        rclcpp::Client<ssp_interfaces::srv::RelayCommand>::SharedFuture &result
    )
    {
        if (mHornFlag and mHornClient->wait_for_service(100ms))
        {
            auto request = std::make_shared<ssp_interfaces::srv::RelayCommand::Request>();
            request->channel = mHornChannel;
            request->state = mHornState;
            result = mHornClient->async_send_request(request).future.share();
            return true;
        }
        return false;
    }

    void Mav2RoboAuxProp::horn_receive(
        rclcpp::FutureReturnCode &return_code, 
        rclcpp::Client<ssp_interfaces::srv::RelayCommand>::SharedFuture &result
    )
    {
        if ((return_code == rclcpp::FutureReturnCode::SUCCESS)
            and (result.get()->state == mHornState))
        {
            mHornFlag = false;
            RCLCPP_INFO(this->get_logger(), "Successfully set horn on channel %d to state %d", mHornChannel, mHornState);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Failed to set horn on channel %d to state %d", mHornChannel, mHornState);
        }
    }

    void Mav2RoboAuxProp::act_ctrl_cb(const mavros_msgs::msg::ActuatorControl &msg)
    {
        if (mThrottleMixGroup == msg.group_mix)
        {
            mThrottleCmd = mThrottleGain * (msg.controls[mThrottleChannel] + mThrottleOffset);
        }
        if (mSteeringMixGroup == msg.group_mix)
        {
            mSteeringCmd = mSteeringGain * (msg.controls[mSteeringChannel] + mSteeringOffset);
        }
        if (mExtendCmdMixGroup == msg.group_mix)
        {
            if (!mExtendCmdFlag and (msg.controls[mExtendCmdChannel] > mExtendThresh)) { mExtendCmdFlag = true; }
            else if (mExtendCmdFlag and (msg.controls[mExtendCmdChannel] < mRetractThresh)) { mExtendCmdFlag = false; }
        }
    }

    void Mav2RoboAuxProp::act_out_stat_cb(const mavros_msgs::msg::ActuatorOutputStatus &msg)
    {
        if (mThrottleMixGroup < 0)
        {
            mThrottleCmd = mThrottleGain * (msg.actuator[mThrottleChannel] + mThrottleOffset);
        }
        if (mSteeringMixGroup < 0)
        {
            mSteeringCmd = mSteeringGain * (msg.actuator[mSteeringChannel] + mSteeringOffset);
        }
        if (mExtendCmdMixGroup < 0)
        {
            if (!mExtendCmdFlag and (msg.actuator[mExtendCmdChannel] > mExtendThresh)) { mExtendCmdFlag = true; }
            else if (mExtendCmdFlag and (msg.actuator[mExtendCmdChannel] < mRetractThresh)) { mExtendCmdFlag = false; }
        }
    }

    void Mav2RoboAuxProp::motor_current_cb(const roboclaw::msg::MotorVoltsAmps &msg)
    {
        if (mLeftRetractIndex == msg.index)
        {
            if (mLeftRetractChannel == 1) { mRetractLeftAmps = fabs(msg.mot1_amps); }
            else { mRetractLeftAmps = fabs(msg.mot2_amps); }
        }
        if (mRightRetractIndex == msg.index)
        {
            if (mRightRetractChannel == 1) { mRetractRightAmps = fabs(msg.mot1_amps); }
            else { mRetractRightAmps = fabs(msg.mot2_amps); }
        }
    }

    void Mav2RoboAuxProp::state_timer_cb()
    {
        aux_propulsion_state_t tmp = mCurrState;
        switch (mCurrState)
        {
            case(aux_propulsion_state_t::BEGIN):
                mCurrState = state_begin_exec();
                break;
            case(aux_propulsion_state_t::INACTIVE):
                mCurrState = state_inactive_exec();
                break;
            case(aux_propulsion_state_t::EXTEND_START):
                mCurrState = state_extend_start_exec();
                break;
            case(aux_propulsion_state_t::EXTEND):
                mCurrState = state_extend_exec();
                break;
            case(aux_propulsion_state_t::ACTIVE_START):
                mCurrState = state_active_start_exec();
                break;
            case(aux_propulsion_state_t::ACTIVE):
                mCurrState = state_active_exec();
                break;
            case(aux_propulsion_state_t::RETRACT_START):
                mCurrState = state_retract_start_exec();
                break;
            case(aux_propulsion_state_t::RETRACT):
                mCurrState = state_retract_exec();
                break;
            default:
                mCurrState = state_begin_exec();
        }
        mLastState = tmp;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_begin_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
        }
        mHornState = false;
        this->mix_motors(0.0, 0.0, false, false);
        this->publish();
        return aux_propulsion_state_t::INACTIVE;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_inactive_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
        }
        mHornState = false;
        this->mix_motors(0.0, 0.0, false, false);
        this->publish();
        if (mExtendCmdFlag) { return aux_propulsion_state_t::EXTEND_START; }
        return aux_propulsion_state_t::INACTIVE;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_extend_start_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
        }
        mRetractLeftComplete = false;
        mRetractRightComplete = false;
        mHornState = true;
        this->mix_motors(0.0, 0.0, false, false);
        this->publish();
        if ((steady_clock::now() - mStateStartTime) > mHornExtendTimeMillis)
        {
            return aux_propulsion_state_t::EXTEND;
        }
        return aux_propulsion_state_t::EXTEND_START;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_extend_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
            mRetractLeftComplete = false;
            mRetractRightComplete = false;
        }
        mHornState = false;
        this->mix_motors(0.0, 0.0, true, false);
        this->publish();
        this->check_current(mExtendThreshAmps);
        if (((steady_clock::now() - mStateStartTime) > mExtendTimeMillis)
            or (mRetractLeftComplete and mRetractRightComplete))
        {
            return aux_propulsion_state_t::ACTIVE_START;
        }
        else if (!mExtendCmdFlag)
        {
            return aux_propulsion_state_t::RETRACT;
        }
        return aux_propulsion_state_t::EXTEND;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_active_start_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
        }
        mHornState = true;
        this->mix_motors(0.0, 0.0, false, false);
        this->publish();
        if ((steady_clock::now() - mStateStartTime) > mHornMotorStartTimeMillis)
        {
            return aux_propulsion_state_t::ACTIVE;
        }
        else if (!mExtendCmdFlag)
        {
            return aux_propulsion_state_t::RETRACT_START;
        }
        return aux_propulsion_state_t::ACTIVE_START;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_active_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
        }
        mHornState = false;
        this->mix_motors(mThrottleCmd, mSteeringCmd, false, false);
        this->publish();
        if (!mExtendCmdFlag)
        {
            return aux_propulsion_state_t::RETRACT_START;
        }
        return aux_propulsion_state_t::ACTIVE;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_retract_start_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
        }
        mHornState = true;
        mRetractLeftComplete = false;
        mRetractRightComplete = false;
        this->mix_motors(0.0, 0.0, false, false);
        this->publish();
        if ((steady_clock::now() - mStateStartTime) > mHornRetractTimeMillis)
        {
            return aux_propulsion_state_t::RETRACT;
        }
        else if (mExtendCmdFlag)
        {
            return aux_propulsion_state_t::ACTIVE_START;
        }
        return aux_propulsion_state_t::RETRACT_START;
    }

    aux_propulsion_state_t Mav2RoboAuxProp::state_retract_exec()
    {
        if (mCurrState != mLastState) 
        { 
            mStateStartTime = steady_clock::now(); 
            mHornFlag = true;
            mRetractLeftComplete = false;
            mRetractRightComplete = false;
        }
        mHornState = false;
        this->mix_motors(0.0, 0.0, false, true);
        this->publish();
        this->check_current(mRetractThreshAmps);
        if (((steady_clock::now() - mStateStartTime) > mHornRetractTimeMillis)
            or (mRetractLeftComplete and mRetractRightComplete))
        {
            return aux_propulsion_state_t::INACTIVE;
        }
        else if (mExtendCmdFlag)
        {
            return aux_propulsion_state_t::EXTEND;
        }
        return aux_propulsion_state_t::RETRACT;
    }

    void Mav2RoboAuxProp::declare_params()
    {

        // create callback lambda
        mParamCBHandle = mParamSub->add_parameter_event_callback(
            [this](const rcl_interfaces::msg::ParameterEvent &e) 
        { if (e.node == string(this->get_fully_qualified_name())) { fetch_params(); } });

        // input parameters
        this->declare_parameter<int8_t>("throttle.mix_group", 0);
        this->declare_parameter<int8_t>("steering.mix_group", 0);
        this->declare_parameter<int8_t>("retract.cmd.mix_group",            0);
        this->declare_parameter<uint8_t>("throttle.channel", 0);
        this->declare_parameter<uint8_t>("steering.channel", 0);
        this->declare_parameter<uint8_t>("retract.cmd.channel", 0);
        this->declare_parameter<float>("retract.cmd.extend_threshold", 0.1);
        this->declare_parameter<float>("retract.cmd.retract_threshold", -0.1);

        // propulsion mix & output parameters
        this->declare_parameter<uint8_t>("propulsion.left.index", 0);
        this->declare_parameter<uint8_t>("propulsion.left.channel", 1);
        this->declare_parameter<uint8_t>("propulsion.right.index", 0);
        this->declare_parameter<uint8_t>("propulsion.right.channel", 1);
        this->declare_parameter<float>("steering.input.gain", 1.0);
        this->declare_parameter<float>("throttle.input.gain", 1.0);
        this->declare_parameter<float>("steering.input.offset", 0.0);
        this->declare_parameter<float>("throttle.input.offset", 0.0);
        this->declare_parameter<float>("propulsion.left.gain", 1.0);
        this->declare_parameter<float>("propulsion.left.offset", 0.0);
        this->declare_parameter<float>("propulsion.right.gain", 1.0);
        this->declare_parameter<float>("propulsion.right.offset", 0.0);
        this->declare_parameter<int32_t>("propulsion.max_output", 32768);
        this->declare_parameter<int32_t>("propulsion.min_output", -32768);

        // extend/retract output parameters 
        this->declare_parameter<uint8_t>("retract.left.index", 0);
        this->declare_parameter<uint8_t>("retract.left.channel", 1);
        this->declare_parameter<uint8_t>("retract.right.index", 0);
        this->declare_parameter<uint8_t>("retract.right.channel", 1);
        this->declare_parameter<int32_t>("retract.out.extend", 0);
        this->declare_parameter<int32_t>("retract.out.retract", 0);
        this->declare_parameter<int32_t>("retract.out.neutral", 0);
        this->declare_parameter<uint8_t>("horn.channel", 0);

        // state transition parameters
        this->declare_parameter<float>("horn.time.retract_start_sec", 0.0);
        this->declare_parameter<float>("horn.time.extend_start_sec", 0.0);
        this->declare_parameter<float>("horn.time.motor_start_sec", 0.0);
        this->declare_parameter<float>("retract.time.retract_sec", 0.0);
        this->declare_parameter<float>("retract.time.extend_sec", 0.0);
        this->declare_parameter<float>("retract.out.retract_stop_amps", 0.0);
        this->declare_parameter<float>("retract.out.extend_stop_amps", 0.0);
        this->declare_parameter<float>("retract.out.shutdown_amps", 0.0);
        this->declare_parameter<float>("step_time_sec", 0.0);
    }

    void Mav2RoboAuxProp::fetch_params()
    {
        this->get_parameter("throttle.mix_group",               mThrottleMixGroup);
        this->get_parameter("steering.mix_group",               mSteeringMixGroup);
        this->get_parameter("retract.cmd.mix_group",            mExtendCmdMixGroup);
        this->get_parameter("throttle.channel",                 mThrottleChannel);
        this->get_parameter("steering.channel",                 mSteeringChannel);
        this->get_parameter("retract.cmd.channel",              mExtendCmdChannel);
        this->get_parameter("retract.cmd.extend_threshold",     mExtendThresh);
        this->get_parameter("retract.cmd.retract_threshold",    mRetractThresh);

        // propulsion mix & output parameter variables
        this->get_parameter("propulsion.left.index",            mLeftMotorIndex);
        this->get_parameter("propulsion.left.channel",          mLeftMotorChannel);
        this->get_parameter("propulsion.right.index",           mRightMotorIndex);
        this->get_parameter("propulsion.right.channel",         mRightMotorChannel);
        this->get_parameter("steering.input.gain",              mSteeringGain);
        this->get_parameter("throttle.input.gain",              mThrottleGain);
        this->get_parameter("steering.input.offset",            mSteeringOffset);
        this->get_parameter("throttle.input.offset",            mThrottleOffset);
        this->get_parameter("propulsion.left.gain",             mLeftMotorGain);
        this->get_parameter("propulsion.left.offset",           mLeftMotorOffset);
        this->get_parameter("propulsion.right.gain",            mRightMotorGain);
        this->get_parameter("propulsion.right.offset",          mRightMotorOffset);
        this->get_parameter("propulsion.max_output",            mMaxMotorOutput);
        this->get_parameter("propulsion.min_output",            mMinMotorOutput);

        // extend/retract output parameter variables
        this->get_parameter("retract.left.index",               mLeftRetractIndex);
        this->get_parameter("retract.left.channel",             mLeftRetractChannel);
        this->get_parameter("retract.right.index",              mRightRetractIndex);
        this->get_parameter("retract.right.channel",            mRightRetractChannel);
        this->get_parameter("retract.out.extend",               mRetractExtendOut);
        this->get_parameter("retract.out.retract",              mRetractRetractOut);
        this->get_parameter("retract.out.neutral",              mRetractNeutralOut);
        this->get_parameter("horn.channel",                     mHornChannel);

        // state transition parameter variables
        this->get_parameter("retract.out.retract_stop_amps",    mRetractThreshAmps);
        this->get_parameter("retract.out.extend_stop_amps",     mExtendThreshAmps);
        this->get_parameter("retract.out.shutdown_amps",        mRetractShutdownThreshAmps);
        
        // turn times in seconds into times in milliseconds...
        float tmp_sec;
        this->get_parameter("horn.time.retract_start_sec",      tmp_sec);
        mHornRetractTimeMillis      = (long int)(tmp_sec * 1000) * 1ms;
        this->get_parameter("horn.time.extend_start_sec",       tmp_sec);
        mHornExtendTimeMillis       = (long int)(tmp_sec * 1000) * 1ms;
        this->get_parameter("horn.time.motor_start_sec",        tmp_sec);
        mHornMotorStartTimeMillis   = (long int)(tmp_sec * 1000) * 1ms;
        this->get_parameter("retract.time.retract_sec",         tmp_sec);
        mRetractTimeMillis          = (long int)(tmp_sec * 1000) * 1ms;
        this->get_parameter("retract.time.extend_sec",          tmp_sec);
        mExtendTimeMillis           = (long int)(tmp_sec * 1000) * 1ms;
        this->get_parameter("step_time_sec",                    tmp_sec);
        mStepTimeMillis             = (long int)(tmp_sec * 1000) * 1ms;
    }

    void Mav2RoboAuxProp::mix_motors(float throttle, float steering, bool extend, bool retract)
    {
        mLeftMotorOutput = bound_val((mLeftMotorGain * (throttle + steering + mLeftMotorOffset)), mMaxMotorOutput, mMinMotorOutput);
        mRightMotorOutput = bound_val((mRightMotorGain * (throttle - steering + mRightMotorOffset)), mMaxMotorOutput, mMinMotorOutput);
        if (extend)
        {
            RCLCPP_INFO(this->get_logger(), "Extending aux propulsion, output %d, completed left %d right %d", mRetractExtendOut, (int)mRetractLeftComplete, (int)mRetractRightComplete);
            mRetractLeftComplete ? mLeftRetractOutput = mRetractNeutralOut : mLeftRetractOutput = mRetractExtendOut;
            mRetractRightComplete ? mRightRetractOutput = mRetractNeutralOut : mRightRetractOutput = mRetractExtendOut;
        }
        else if (retract)
        {
            mRetractLeftComplete ? mLeftRetractOutput = mRetractNeutralOut : mLeftRetractOutput = mRetractRetractOut;
            mRetractRightComplete ? mRightRetractOutput = mRetractNeutralOut : mRightRetractOutput = mRetractRetractOut;
        }
        else 
        {
            mLeftRetractOutput = mRetractNeutralOut;
            mRightRetractOutput = mRetractNeutralOut;
        }
    }

    void Mav2RoboAuxProp::check_current(float limit)
    {
        if (mRetractLeftAmps > limit) { mRetractLeftComplete = true; }
        if (mRetractRightAmps > limit) { mRetractRightComplete = true; }
    }

    void Mav2RoboAuxProp::publish()
    {
        auto left_mot_msg           = roboclaw::msg::MotorDutySingle();
        auto right_mot_msg          = roboclaw::msg::MotorDutySingle();
        auto left_retract_msg       = roboclaw::msg::MotorDutySingle();
        auto right_retract_msg      = roboclaw::msg::MotorDutySingle();
        left_mot_msg.index          = mLeftMotorIndex;
        left_mot_msg.channel        = mLeftMotorChannel;
        left_mot_msg.mot_duty       = mLeftMotorOutput;
        right_mot_msg.index         = mRightMotorIndex;
        right_mot_msg.channel       = mRightMotorChannel;
        right_mot_msg.mot_duty      = mRightMotorOutput;
        left_retract_msg.index      = mLeftRetractIndex;
        left_retract_msg.channel    = mLeftRetractChannel;
        left_retract_msg.mot_duty   = mLeftRetractOutput;
        right_retract_msg.index     = mRightRetractIndex;
        right_retract_msg.channel   = mRightRetractChannel;
        right_retract_msg.mot_duty  = mRightRetractOutput;
        mLeftMotorPub->publish(left_mot_msg);
        mRightMotorPub->publish(right_mot_msg);
        mLeftRetractPub->publish(left_retract_msg);
        mRightRetractPub->publish(right_retract_msg);
    }


} // namespace mav2robo

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mav2robo::Mav2RoboAuxProp>("mav2roboclaw_aux_propulsion");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rclcpp::Client<ssp_interfaces::srv::RelayCommand>::SharedFuture result;
        if(node->horn_prepare(result))
        {
            rclcpp::FutureReturnCode code = rclcpp::spin_until_future_complete(node, result);
            node->horn_receive(code, result);
        }
        // This is a stupid ugly hack to keep this thread from slamming the CPU it's running on -- PN 9/26/22
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    rclcpp::shutdown();

    return 0;
}