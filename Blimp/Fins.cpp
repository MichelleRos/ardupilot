#include "Blimp.h"

#include <SRV_Channel/SRV_Channel.h>

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo Fins::var_info[] = {

    // @Param: FREQ_HZ
    // @DisplayName: Fins frequency
    // @Description: This is the oscillation frequency of the fins
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("FREQ_HZ", 1, Fins, freq_hz, 3),

    // @Param: TURBO_MODE
    // @DisplayName: Enable turbo mode
    // @Description: Enables double speed on high offset.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TURBO_MODE", 2, Fins, turbo_mode, 0),

    // @Param: THR_MAX
    // @DisplayName: Maximum throttle
    // @Description: Maximum throttle allowed. Constrains any throttle input to this value (negative and positive) Set it to 1 to disable (i.e. allow max throttle).
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX", 3, Fins, thr_max, 1),

    // @Param: RP_DAMP_LIM
    // @DisplayName: Roll/Pitch limit before damping outputs, in degrees. Zero means disabled.
    // @Description: RP D
    // @Range: 0 180
    // @User: Standard
    AP_GROUPINFO("RP_DAMP_LIM", 4, Fins, rp_damp_lim, 0),

    // @Param: RP_DAMP_OFF
    // @DisplayName: Roll/Pitch limit where outputs are fully cut off when _AMT is 1.
    // @Description: RP D
    // @Range: 0 180
    // @User: Standard
    AP_GROUPINFO("RP_DAMP_OFF", 5, Fins, rp_damp_off, 0),

    // @Param: RP_DAMP_AMT
    // @DisplayName: Roll/Pitch output damping amount. Higher number means more damping.
    // @Description: RP D
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("RP_DAMP_AMT", 9, Fins, rp_damp_amt, 1),

    // @Param: RP_DAMP_LI2
    // @DisplayName: Roll/Pitch rate limit before damping outputs, in degrees. Zero means disabled.
    // @Description: RP D
    // @Range: 0 180
    // @User: Standard
    AP_GROUPINFO("RP_DAMP_LI2", 6, Fins, rp_damp_lim2, 0),

    // @Param: RP_DAMP_OF2
    // @DisplayName: Roll/Pitch rate limit where outputs are fully cut off when _AMT is 1.
    // @Description: RP D
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("RP_DAMP_OF2", 7, Fins, rp_damp_off2, 0),

    // @Param: RP_DAMP_AM2
    // @DisplayName: Roll/Pitch output damping amount. Higher number means more damping.
    // @Description: RP D
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("RP_DAMP_AM2", 10, Fins, rp_damp_amt2, 1),

    // @Param: RP_DAMP_MSK
    // @DisplayName: Roll/Pitch output damping mask for which outputs to affect
    // @Description: RP D
    // @Values: 15:All enabled, 3:Right & Front only
    // @Bitmask: 0:Right,1:Front,2:Down,3:Yaw
    // @User: Standard
    AP_GROUPINFO("RP_DAMP_MSK", 8, Fins, rp_damp_msk, 15),

    AP_GROUPEND
};

//constructor
Fins::Fins(uint16_t loop_rate, motor_frame_class frame) :
    _loop_rate(loop_rate),
    _frame(frame)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void Fins::setup_finsmotors()
{
    switch ((Fins::motor_frame_class)_frame) {
        case Fins::MOTOR_FRAME_FISHBLIMP:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Setting up FishBlimp.");
            setup_fins();
            break;
        case Fins::MOTOR_FRAME_FOUR_MOTOR:
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Setting up FourMotor.");
            setup_motors();
            break;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ERROR: Wrong frame class.");
            break;
    }
}

void Fins::setup_fins()
{
    //fin   #   r   f   d     y,    r   f     d     y
    //right, front, down, yaw for amplitude then for offset
    add_fin(0,  0,  1, 0.5,   0,    0,  0,  0.5,    0); //Back
    add_fin(1,  0, -1, 0.5,   0,    0,  0,  0.5,    0); //Front
    add_fin(2, -1,  0,   0, 0.5,    0,  0,    0,  0.5); //Right
    add_fin(3,  1,  0,   0, 0.5,    0,  0,    0, -0.5); //Left

    SRV_Channels::set_angle(SRV_Channel::k_motor1, RC_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, RC_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, RC_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, RC_SCALE);
}

void Fins::setup_motors()
{   //   motor#   r   f   d    y
    add_motor(0,  0,  1,  0,   1); //FrontLeft
    add_motor(1,  0,  1,  0,  -1); //FrontRight
    add_motor(2,  0,  0, -1,   0); //Up
    add_motor(3,  1,  0,   0,  0); //Right

    SRV_Channels::set_angle(SRV_Channel::k_motor1, RC_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, RC_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, RC_SCALE);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, RC_SCALE);
}

void Fins::add_fin(int8_t fin_num, float right_amp_fac, float front_amp_fac, float down_amp_fac, float yaw_amp_fac,
                   float right_off_fac, float front_off_fac, float down_off_fac, float yaw_off_fac)
{
    // ensure valid fin number is provided
    if (fin_num >= 0 && fin_num < NUM_FINS) {

        // set amplitude factors
        _right_amp_factor[fin_num] = right_amp_fac;
        _front_amp_factor[fin_num] = front_amp_fac;
        _down_amp_factor[fin_num] = down_amp_fac;
        _yaw_amp_factor[fin_num] = yaw_amp_fac;

        // set offset factors
        _right_off_factor[fin_num] = right_off_fac;
        _front_off_factor[fin_num] = front_off_fac;
        _down_off_factor[fin_num] = down_off_fac;
        _yaw_off_factor[fin_num] = yaw_off_fac;
    }
}

void Fins::add_motor(int8_t fin_num, float right_amp_fac, float front_amp_fac, float down_amp_fac, float yaw_amp_fac)
{
    // ensure valid fin number is provided
    if (fin_num >= 0 && fin_num < NUM_FINS) {
        _right_amp_factor[fin_num] = right_amp_fac;
        _front_amp_factor[fin_num] = front_amp_fac;
        _down_amp_factor[fin_num] = down_amp_fac;
        _yaw_amp_factor[fin_num] = yaw_amp_fac;
    }
}

//B,F,R,L = 0,1,2,3
void Fins::output()
{
    if (!_armed) {
        // set everything to zero so fins stop moving
        right_out = 0;
        front_out = 0;
        down_out  = 0;
        yaw_out   = 0;
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_FINI(right_out, front_out, down_out, yaw_out);
#endif

    //Constrain after logging so as to still show when sub-optimal tuning is causing massive overshoots.

    right_out = constrain_float(right_out, -thr_max, thr_max);
    front_out = constrain_float(front_out, -thr_max, thr_max);
    down_out = constrain_float(down_out, -thr_max, thr_max);
    yaw_out = constrain_float(yaw_out, -thr_max, thr_max);

    blimp.Write_FINM(right_out, front_out, down_out, yaw_out);

    float aroll = fabsf(blimp.ahrs.get_roll());
    float apitch = fabsf(blimp.ahrs.get_pitch());
    Vector3f agyro = blimp.ahrs.get_gyro();

    //Damping by roll & pitch angle
    if (rp_damp_off > 0 && (rp_damp_off > rp_damp_lim)) //Disable when unused, and also protect against divide by zero
    {
        float excessr = 0;
        float excessp = 0;
        if (fabsf(aroll) > radians(rp_damp_lim)) excessr = (fabsf(aroll)-radians(rp_damp_lim))/(radians(rp_damp_off)-radians(rp_damp_lim));
        if (fabsf(apitch) > radians(rp_damp_lim)) excessp = (fabsf(apitch)-radians(rp_damp_lim))/(radians(rp_damp_off)-radians(rp_damp_lim));
/*
        actual, lim, off
        (a-lim)/(off-lim)
        eg
        21-20 / 30-20 = 1/10 = 0.1
        25-20 / 30-20 = 5/10 = 0.5
        30-20 / 30-20 = 10/10 = 1
        Thus need 1-ans.
*/
        float rp_scale = 0;
        //Cut off if it's past rp_damp_off
        if ((rp_damp_amt*(excessr+excessp)) >= 1) rp_scale = 0;
        else rp_scale = 1 - rp_damp_amt*(excessr+excessp);

        AP::logger().WriteStreaming("FIND", "TimeUS,er,ep,rps", "Qfff",
                                AP_HAL::micros64(),
                                excessr,
                                excessp,
                                rp_scale);
        // gcs().send_named_float("EECS", rp_scale);

        if (rp_damp_msk & (1<<(1-1))) right_out = right_out * rp_scale;
        if (rp_damp_msk & (1<<(2-1))) front_out = front_out * rp_scale;
        if (rp_damp_msk & (1<<(3-1))) down_out = down_out * rp_scale;
        if (rp_damp_msk & (1<<(4-1))) yaw_out = yaw_out * rp_scale;
    }

    //Damping by roll & pitch rate
    if (rp_damp_off2 > 0 && (rp_damp_off2 > rp_damp_lim2)) //Disable when unused, and also protect against divide by zero
    {
        float excessr2 = 0;
        float excessp2 = 0;
        if (fabsf(agyro.x) > radians(rp_damp_lim2)) excessr2 = (fabsf(agyro.x)-radians(rp_damp_lim2))/(radians(rp_damp_off2)-radians(rp_damp_lim2));
        if (fabsf(agyro.y) > radians(rp_damp_lim2)) excessp2 = (fabsf(agyro.y)-radians(rp_damp_lim2))/(radians(rp_damp_off2)-radians(rp_damp_lim2));

        float rp_scale2 = 0;
        //Cut off if it's past rp_damp_off
        if ((rp_damp_amt*(excessr2+excessp2)) >= 1) rp_scale2 = 0;
        else rp_scale2 = 1 - rp_damp_amt*(excessr2+excessp2);

        AP::logger().WriteStreaming("FIN2", "TimeUS,er,ep,rps", "Qfff",
                                AP_HAL::micros64(),
                                excessr2,
                                excessp2,
                                rp_scale2);

        if (rp_damp_msk & (1<<(1-1))) right_out = right_out * rp_scale2;
        if (rp_damp_msk & (1<<(2-1))) front_out = front_out * rp_scale2;
        if (rp_damp_msk & (1<<(3-1))) down_out = down_out * rp_scale2;
        if (rp_damp_msk & (1<<(4-1))) yaw_out = yaw_out * rp_scale2;
    }

    blimp.Write_FINN(right_out, front_out, down_out, yaw_out);

    switch ((Fins::motor_frame_class)_frame) {
        case Fins::MOTOR_FRAME_FISHBLIMP:
            output_fins();
            break;
        case Fins::MOTOR_FRAME_FOUR_MOTOR:
            output_motors();
            break;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ERROR: Wrong frame class.");
            break;
    }
}

void Fins::output_fins()
{
    _time = AP_HAL::micros() * 1.0e-6;

    for (int8_t i=0; i<NUM_FINS; i++) {
        _amp[i] =  fmaxf(0,_right_amp_factor[i]*right_out) + fmaxf(0,_front_amp_factor[i]*front_out) +
                   fabsf(_down_amp_factor[i]*down_out) + fabsf(_yaw_amp_factor[i]*yaw_out);
        _off[i] = _right_off_factor[i]*right_out + _front_off_factor[i]*front_out +
                  _down_off_factor[i]*down_out + _yaw_off_factor[i]*yaw_out;
        _freq[i] = 1;

        _num_added = 0;
        if (fmaxf(0,_right_amp_factor[i]*right_out) > 0.0f) {
            _num_added++;
        }
        if (fmaxf(0,_front_amp_factor[i]*front_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_down_amp_factor[i]*down_out) > 0.0f) {
            _num_added++;
        }
        if (fabsf(_yaw_amp_factor[i]*yaw_out) > 0.0f) {
            _num_added++;
        }

        if (_num_added > 0) {
            _off[i] = _off[i]/_num_added; //average the offsets
        }

        if ((_amp[i]+fabsf(_off[i])) > thr_max) {
            _amp[i] = thr_max - fabsf(_off[i]);
        }

        if (turbo_mode) {
            //double speed fins if offset at max...
            if (_amp[i] <= 0.6 && fabsf(_off[i]) >= 0.4) {
                _freq[i] = 2;
            }
        }
        // finding and outputting current position for each servo from sine wave
        _thrpos[i]= _amp[i]*cosf(freq_hz * _freq[i] * _time * 2 * M_PI) + _off[i];
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _thrpos[i] * RC_SCALE);
    }

#if HAL_LOGGING_ENABLED
    blimp.Write_FINO(_amp, _off);
#endif
}

void Fins::output_motors()
{
    for (int8_t i=0; i<NUM_FINS; i++) {
        //Calculate throttle for each motor
        _thrpos[i] = constrain_float(_right_amp_factor[i]*right_out + _front_amp_factor[i]*front_out + _down_amp_factor[i]*down_out + _yaw_amp_factor[i]*yaw_out, -thr_max, thr_max);

        //Set output
        SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _thrpos[i] * RC_SCALE);
    }

}

void Fins::output_min()
{
    right_out = 0;
    front_out = 0;
    down_out  = 0;
    yaw_out   = 0;
    Fins::output();
}

const char* Fins::get_frame_string()
{
    switch ((Fins::motor_frame_class)_frame) {
        case Fins::MOTOR_FRAME_FISHBLIMP:
            return "FISHBLIMP";
            break;
        case Fins::MOTOR_FRAME_FOUR_MOTOR:
            return "FOURMOTOR";
            break;
        default:
            return "NOFRAME";
            break;
    }



}

// float constrain_float_ratio(const float amt, const float low, const float high, float &rat)//, uint32_t line)
// {
//     // the check for NaN as a float prevents propagation of floating point
//     // errors through any function that uses constrain_value(). The normal
//     // float semantics already handle -Inf and +Inf
// //     if (isnan(amt)) {
// // #if AP_INTERNALERROR_ENABLED
// //         AP::internalerror().error(AP_InternalError::error_t::constraining_nan, line);
// // #endif
// //         return (low + high) / 2;
// //     }
//
//     if (amt < low) {
//         if (low != 0.0f) {
//             rat = amt/low;
//             return low;
//         } else {
//             rat = 1.0f;
//             return low;
//         }
//     }
//
//     if (amt > high ) {
//         if (high != 0.0f) {
//             rat = amt/high;
//             return high;
//         } else {
//             rat = 1.0f;
//             return high;
//         }
//     }
//
//     return amt;
// }