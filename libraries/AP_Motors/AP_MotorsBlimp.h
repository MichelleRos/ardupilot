/// @file	AP_MotorsBlimp.h
/// @brief	Motor control class for Motor blimp
#pragma once

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_Motors_Class.h"

#define AP_MOTORS_BLIMP_SPEED_DEFAULT 50 //Update rate
#define MOTOR_SCALE_MAX 1000
#define NUM_MOTORS 4

/// @class      AP_MotorsBlimp
class AP_MotorsBlimp : public AP_Motors {
public:

    /// Constructor
    AP_MotorsBlimp(uint16_t speed_hz = AP_MOTORS_BLIMP_SPEED_DEFAULT) :
        AP_Motors(speed_hz)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // init
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;

    void set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override {
        //MIR: Add check for not armed
        _frame_class = frame_class;
        _frame_type = frame_type;
    }

    // set update rate to motors
    void set_update_rate( uint16_t speed_hz ) override {
        _speed_hz = speed_hz;
    }

    void AP_MotorsBlimp::update_throttle_filter() { } //So the compiler doesn't complain about virtual function...

    float get_throttle_hover() const override { return _throttle_hover; }

    uint32_t AP_MotorsBlimp::get_motor_mask() {
        //This is what Multicopter does...
        return SRV_Channels::get_output_channel_mask(SRV_Channel::k_boost_throttle); 
    } 

    // output_min - sets servos to neutral point with motors stopped
    void output_min() override;

    // output - sends commands to the motors
    void output() override;

    // Run arming checks
    bool arming_checks(size_t buflen, char *buffer) const override;

    // Tell user motor test is disabled on heli
    bool motor_test_checks(size_t buflen, char *buffer) const override;

    // output_test_seq - disabled on heli, do nothing
    void _output_test_seq(uint8_t motor_seq, int16_t pwm) override {};

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    //input right movement, negative for left, +1 to -1
    float               right_out;
    //input front/forwards movement, negative for backwards, +1 to -1
    float               front_out;
    //input yaw, positive for clockwise, negative for counterclockwise, +1 to -1
    float               yaw_out;
    //input height control, negative for up, +1 to -1
    float               down_out;

protected:

    // output - sends commands to the motors
    void output_armed_stabilizing() override;
    void output_armed_zero_throttle();
    void output_disarmed();

    void setup_motorsfins();
    void add_motor(int8_t motor_num, float right_fac, float front_fac, float down_fac, float yaw_fac);

    float               _thrpos[NUM_MOTORS]; //throttle output or servo position
    float               _right_amp_factor[NUM_MOTORS];
    float               _front_amp_factor[NUM_MOTORS];
    float               _down_amp_factor[NUM_MOTORS];
    float               _yaw_amp_factor[NUM_MOTORS];

    // run spool logic
    void                output_logic();

    const char* _get_frame_string() const override { return "HELI"; }

    AP_Float        _tmp;

    uint16_t _speed_hz;
    float _throttle_hover;

    motor_frame_type _frame_type;
    motor_frame_class _frame_class;
};
