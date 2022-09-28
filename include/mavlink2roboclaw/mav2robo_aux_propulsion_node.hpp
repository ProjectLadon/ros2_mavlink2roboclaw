// Copyright (c) 2022 Ladon Robotics
// Author: Pierce Nichols <pierce@ladonrobotics.com>

#pragma once

#include <string>
#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>

#include "mavlink2roboclaw/mav2robo_common.hpp"

#include "rclcpp/rclcpp.hpp"

#include "roboclaw/msg/motor_volts_amps.hpp"
#include "roboclaw/msg/motor_duty_single.hpp"
#include "ssp_interfaces/srv/relay_command.hpp"
#include "mavros_msgs/msg/actuator_control.hpp"
#include "mavros_msgs/msg/actuator_output_status.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std;

namespace mav2robo
{

    enum class aux_propulsion_state_t : uint8_t
    {
        BEGIN = 0,
        INACTIVE,
        EXTEND_START,
        EXTEND,
        ACTIVE_START,
        ACTIVE,
        RETRACT_START,
        RETRACT
    };

    class Mav2RoboAuxProp : public rclcpp::Node
    {
    public:
        Mav2RoboAuxProp(string name);
        
        // public methods
        bool horn_prepare(rclcpp::Client<ssp_interfaces::srv::RelayCommand>::SharedFuture &result);
        void horn_receive(rclcpp::FutureReturnCode &return_code, rclcpp::Client<ssp_interfaces::srv::RelayCommand>::SharedFuture &result);

    private:
        // input parameter variables
        int8_t      mThrottleMixGroup;      // if this is less than 0, then use ActuatorOutputStatus inputs
        int8_t      mSteeringMixGroup;      // if this is less than 0, then use ActuatorOutputStatus inputs
        int8_t      mExtendCmdMixGroup;     // if this is less than 0, then use ActuatorOutputStatus inputs
        uint8_t     mThrottleChannel;       
        uint8_t     mSteeringChannel;
        uint8_t     mExtendCmdChannel;
        float       mExtendThresh;
        float       mRetractThresh;

        // propulsion mix & output parameter variables
        uint8_t     mLeftMotorIndex;
        uint8_t     mLeftMotorChannel;
        uint8_t     mRightMotorIndex;
        uint8_t     mRightMotorChannel;
        float       mSteeringGain;
        float       mThrottleGain;
        float       mSteeringOffset;
        float       mThrottleOffset;
        float       mLeftMotorGain;
        float       mRightMotorGain;
        float       mLeftMotorOffset;
        float       mRightMotorOffset;
        int32_t     mMaxMotorOutput;
        int32_t     mMinMotorOutput;


        // extend/retract output parameter variables
        uint8_t                     mLeftRetractIndex;
        uint8_t                     mLeftRetractChannel;
        uint8_t                     mRightRetractIndex;
        uint8_t                     mRightRetractChannel;
        int32_t                     mRetractExtendOut;
        int32_t                     mRetractRetractOut;
        int32_t                     mRetractNeutralOut;
        uint8_t                     mHornChannel;

        // state transition parameter variables
        milliseconds                mHornRetractTimeMillis;
        milliseconds                mHornExtendTimeMillis;
        milliseconds                mHornMotorStartTimeMillis;
        milliseconds                mRetractTimeMillis;
        milliseconds                mExtendTimeMillis;
        float                       mRetractThreshAmps;
        float                       mExtendThreshAmps;
        float                       mRetractShutdownThreshAmps;
        milliseconds                mStepTimeMillis;

        // state variables
        aux_propulsion_state_t      mCurrState;
        aux_propulsion_state_t      mLastState;
        bool                        mHornFlag;
        bool                        mHornState;
        bool                        mExtendCmdFlag;
        time_point<steady_clock>    mStateStartTime;
        float                       mThrottleCmd;
        float                       mSteeringCmd;
        float                       mRetractLeftAmps;
        float                       mRetractRightAmps;
        bool                        mRetractLeftComplete;
        bool                        mRetractRightComplete;

        // output variables
        int32_t                     mLeftMotorOutput;
        int32_t                     mRightMotorOutput;
        int32_t                     mLeftRetractOutput;
        int32_t                     mRightRetractOutput;

        // service client
        rclcpp::Client<ssp_interfaces::srv::RelayCommand>::SharedPtr mHornClient;

        // parameter update subscriber & callback
        shared_ptr<rclcpp::ParameterEventHandler>               mParamSub;
        rclcpp::ParameterEventCallbackHandle::SharedPtr         mParamCBHandle;

        // subscriptions
        rclcpp::Subscription<mavros_msgs::msg::ActuatorControl>::SharedPtr      mActCtrlSub;
        rclcpp::Subscription<mavros_msgs::msg::ActuatorOutputStatus>::SharedPtr mActOutStatSub;
        rclcpp::Subscription<roboclaw::msg::MotorVoltsAmps>::SharedPtr          mMotorCurrentSub;

        // publications
        rclcpp::Publisher<roboclaw::msg::MotorDutySingle>::SharedPtr        mRightMotorPub;
        rclcpp::Publisher<roboclaw::msg::MotorDutySingle>::SharedPtr        mLeftMotorPub;
        rclcpp::Publisher<roboclaw::msg::MotorDutySingle>::SharedPtr        mRightRetractPub;
        rclcpp::Publisher<roboclaw::msg::MotorDutySingle>::SharedPtr        mLeftRetractPub;

        // timers
        rclcpp::TimerBase::SharedPtr mStateTimer;

        // callbacks
        void act_ctrl_cb(const mavros_msgs::msg::ActuatorControl &msg);
        void act_out_stat_cb(const mavros_msgs::msg::ActuatorOutputStatus &msg);
        void motor_current_cb(const roboclaw::msg::MotorVoltsAmps &msg);
        void state_timer_cb();

        // state functions
        aux_propulsion_state_t state_begin_exec();
        aux_propulsion_state_t state_inactive_exec();
        aux_propulsion_state_t state_extend_start_exec();
        aux_propulsion_state_t state_extend_exec();
        aux_propulsion_state_t state_active_start_exec();
        aux_propulsion_state_t state_active_exec();
        aux_propulsion_state_t state_retract_start_exec();
        aux_propulsion_state_t state_retract_exec();

        // internal methods
        void declare_params();
        void fetch_params();
        void mix_motors(float throttle, float steering, bool extend, bool retract);
        void publish();
        inline int32_t bound_val(int32_t in, int32_t max, int32_t min) { return std::max(min, std::min(in, max)); }

    };
} // namespace mav2robo