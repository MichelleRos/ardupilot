/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsBlimp.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsBlimp::var_info[] = {

    // @Param: TMP
    // @DisplayName: name
    // @Description: desc
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TMP", 3, AP_MotorsBlimp, _tmp, 0),

    AP_GROUPEND
};

// init/setup
void AP_MotorsBlimp::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remember frame class and type
    _frame_type = frame_type;
    _frame_class = frame_class;

    if (_frame_class != MOTOR_FRAME_BLIMP) {
        //Shouldn't reach this.
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ERROR: Wrong frame class.");
    }

    // set update rate
    set_update_rate(_speed_hz);
}

// setup motors/fins
void AP_MotorsBlimp::setup_motorsfins()
{
    //        fin #, right, front, down, yaw
    add_motor(0,     0,     1,     0.5,  0); //Back
    add_motor(1,     0,     -1,    0.5,  0); //Front
    add_motor(2,    -1,     0,     0,   0.5); //Right
    add_motor(3,     1,     0,     0,   0.5); //Left

    SRV_Channels::set_angle(SRV_Channel::k_motor1, MOTOR_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, MOTOR_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, MOTOR_SCALE_MAX);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, MOTOR_SCALE_MAX);
}

void AP_MotorsBlimp::add_motor(int8_t motor_num, float right_fac, float front_fac, float down_fac, float yaw_fac)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < NUM_MOTORS) {
        _right_amp_factor[motor_num] = right_fac;
        _front_amp_factor[motor_num] = front_fac;
        _down_amp_factor[motor_num] = down_fac;
        _yaw_amp_factor[motor_num] = yaw_fac;
    } else {
        //Shouldn't reach this.
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ERROR: Wrong motor number.");
    }
}


// output - sends commands to the servos
void AP_MotorsBlimp::output()
{
    if (!armed()) {
        // set everything to zero so motors stop spinning
        right_out = 0;
        front_out = 0;
        down_out  = 0;
        yaw_out   = 0;
    }

    for (int8_t i=0; i<NUM_MOTORS; i++) {
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _thrpos[i] * MOTOR_SCALE_MAX);
    }

};

// output_min - sets servos to neutral point with motors stopped
void AP_MotorsBlimp::output_min()
{
    // set everything to zero so motors stop spinning
    right_out = 0;
    front_out = 0;
    down_out  = 0;
    yaw_out   = 0;

    for (int8_t i=0; i<NUM_MOTORS; i++) {
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _thrpos[i] * MOTOR_SCALE_MAX);
    }
}
