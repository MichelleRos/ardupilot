#include "AP_Quicktune.h"

#if QUICKTUNE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <RC_Channel/RC_Channel.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>

// #pragma GCC diagnostic ignored "-Wno-narrowing"


const AP_Param::GroupInfo AP_Quicktune::var_info[] = {
    // @Param: QUIK_ENABLE
    // @DisplayName: Enable Quicktune
    // @Description:
    // @User: Standard
    AP_GROUPINFO("ENABLE", 0, AP_Quicktune, enable, 0),

    // @Param: QUIK_AXES
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AP_Quicktune, axes_enabled, 7),

    // @Param: QUIK_ENABLE
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("DOUBLE_TIME", 2, AP_Quicktune, double_time, 10),

    // @Param: QUIK_GAIN_MARGIN
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("GAIN_MARGIN", 3, AP_Quicktune, gain_margin, 60),

    // @Param: QUIK_OSC_SMAX
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("OSC_SMAX", 4, AP_Quicktune, osc_smax, 5),

    // @Param: QUIK_YAW_P_MAX
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("YAW_P_MAX", 5, AP_Quicktune, yaw_p_max, 0.5),

    // @Param: QUIK_YAW_D_MAX
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("YAW_D_MAX", 6, AP_Quicktune, yaw_d_max, 0.01),

    // @Param: QUIK_RP_PI_RATIO
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("RP_PI_RATIO", 7, AP_Quicktune, rp_pi_ratio, 1.0),

    // @Param: QUIK_Y_PI_RATIO
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("Y_PI_RATIO", 8, AP_Quicktune, y_pi_ratio, 10),

    // @Param: QUIK_AUTO_FILTER
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("AUTO_FILTER", 9, AP_Quicktune, auto_filter, 1),

    // @Param: QUIK_AUTO_SAVE
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("AUTO_SAVE", 10, AP_Quicktune, auto_save, 0),

    // @Param: QUIK_MAX_REDUCE
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("MAX_REDUCE", 12, AP_Quicktune, max_reduce, 20),

    // @Param: QUIK_OPTIONS
    // @DisplayName:
    // @Description:
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 13, AP_Quicktune, options, 0),

    AP_GROUPEND
};

//Need access to roll, pitch yaw rc channels, and gyro filter param
//Currently .just. doing this for multicopter (since they are the most likely to be running eg 1MB boards that can't do scripting)

//Call at loop rate
void AP_Quicktune::update(){
    // attitude_control.get_rate_roll_pid().kD(


    if (enable < 1){
        return;
    }
    if (have_pilot_input()){
        last_pilot_input = get_time();
    }
    uint8_t pos;
    bool sw_pos = rc().get_aux_cached(RC_Channel::AUX_FUNC::SCRIPTING_1, pos);
    if (!sw_pos){
        return;
    }
    int8_t sw_pos_tune = 1;
    int8_t sw_pos_save = 2;
    if ((options & OPTIONS_TWO_POSITION) != 0){
        sw_pos_tune = 2;
        sw_pos_save = -1;
    }
    if (sw_pos == sw_pos_tune && (!arming->is_armed() || !vehicle->get_likely_flying()) && get_time() > last_warning + 5){
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Tuning: Must be flying to tune");
        last_warning = get_time();
        return;
    }
    if (sw_pos == 0 || !AP::arming().is_armed() || !vehicle->get_likely_flying()){
        //-- abort, revert parameters
        if (need_restore){
            need_restore = false;
            restore_all_params();
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Tuning: reverted");
            tune_done_time = 0;//nil
        }
        reset_axes_done();
        return;
    }
    if (sw_pos == sw_pos_save){
        // -- save all params
        if (need_restore){
            need_restore = false;
            save_all_params();
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: saved");
        }
    }
    if (sw_pos != sw_pos_tune){
        return;
    }

    if (get_time() - last_stage_change < STAGE_DELAY){
        //update_slew_gain(); (was a function, but only called once)
        if (slew_parm != param_s::END){
            float P = get_param(slew_parm);
            // local axis = param_axis(slew_parm)
            // local ax_stage = string.sub(slew_parm, -1)
            adjust_gain(slew_parm, P+slew_delta);
            slew_steps = slew_steps - 1;
            // logger.write('QUIK','SRate,Gain,Param', 'ffn', get_slew_rate(axis), P:get(), axis .. ax_stage)
            if (slew_steps == 0){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f", slew_parm, P);
                slew_parm = param_s::END;
                if (get_current_axis() == axis_names::DONE){
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: DONE");
                    tune_done_time = get_time();
                }
            }
        }
        return;
    }

    axis_names axis = get_current_axis();
    if (axis == axis_names::DONE){
        // -- nothing left to do, check autosave time
        if (tune_done_time != 0 and auto_save > 0){
            if (get_time() - tune_done_time > auto_save){
                need_restore = false;
                save_all_params();
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: saved");
                tune_done_time = 0;
            }
        }
        return;
    }

    if (!need_restore){
        // -- we are just starting tuning, get current values
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: starting tune");
        get_all_params();
        setup_SMAX();
    }

    if (get_time() - last_pilot_input < PILOT_INPUT_DELAY){
        return;
    }

    if (!filter_done(axis)){
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting %s tune", axis);
        setup_filters(axis);
    }

    float srate = get_slew_rate(axis);
    param_s pname = get_pname(axis, stage);
    float P = get_param(pname);
    float oscillating = srate > osc_smax;
    float limited = reached_limit(pname, P);
    if (limited || oscillating){
        float reduction = (100.0-gain_margin)*0.01;
        if (!oscillating){
            reduction = 1.0;
        }
        float new_gain = P * reduction;
        float limit = gain_limit(pname);
        if (limit > 0.0 && new_gain > limit){
            new_gain = limit;
        }
        float old_gain = param_saved[uint8_t(pname)];
        if (new_gain < old_gain && (pname == param_s::PIT_D || pname == param_s::RLL_D)){
            //-- we are lowering a D gain from the original gain. Also lower the P gain by the same amount so that we don't trigger P oscillation. We don't drop P by more than a factor of 2
            float ratio = fmaxf(new_gain / old_gain, 0.5);
            param_s P_name = param_s(uint8_t(pname)+2); //from P to D
            float old_P = get_param(P_name);;
            float new_P = old_P * ratio;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "adjusting %s %.3f -> %.3f", P_name, old_P, new_P);
            adjust_gain_limited(P_name, new_P);
        }
        setup_slew_gain(pname, new_gain);
        logger->WriteStreaming("QUIK","TimeUS,SRate,Gain,Param", "QffI", AP_HAL::micros64(), srate, P, int(pname));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Tuning: %s done", pname);
        advance_stage(axis);
        last_stage_change = get_time();
    } else {
        float new_gain = P*get_gain_mul();
        if (new_gain <= 0.0001){
            new_gain = 0.001;
        }
        adjust_gain_limited(pname, new_gain);
        logger->WriteStreaming("QUIK","TimeUS,SRate,Gain,Param", "QffI", AP_HAL::micros64(), srate, P, int(pname));
        if (get_time() - last_gain_report > 3){
            last_gain_report = get_time();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f sr:%.2f", pname, new_gain, srate);
        }
    }
}

//Need to remeber starting params & be able to reset them

void AP_Quicktune::reset_axes_done()
{
//Reset the parameter for which axes have been done.
}

void AP_Quicktune::setup_SMAX()
{
//Check each SMAX param, set to DEFAULT_SMAX if it is zero.
}

void AP_Quicktune::setup_filters(AP_Quicktune::axis_names axis)
{
//Set filters for FLTD, FLTT to INS_GYRO_FILTER * FLTT_MUL or FLTD_MUL.
//Set FLTE to YAW_FLTE_MAX if it is 0 or greater than that.
}

bool AP_Quicktune::have_pilot_input()
{
//Check whether there is pilot input currently.
}

AP_Quicktune::axis_names AP_Quicktune::get_current_axis()
{
    // get the axis name we are working on, or DONE for all done 
    axis_names axis_name;
    for (int8_t i = 1; i < int8_t(axis_names::DONE); i++){
        if (axis_enabled(i) == true && axis_done(axis_name) == false){
            return axis_name;
        }
    }
    return axis_names::DONE;
}

float AP_Quicktune::get_slew_rate(AP_Quicktune::axis_names axis)
{
    switch(axis) {
    case axis_names::RLL:
        return attitude_control.get_rate_roll_pid().get_pid_info().slew_rate;
        break;
    case axis_names::PIT:
        return attitude_control.get_rate_pitch_pid().get_pid_info().slew_rate;
        break;
    case axis_names::YAW:
        return attitude_control.get_rate_yaw_pid().get_pid_info().slew_rate;
        break;
    default:
        return 0.0;
    }
}

int8_t AP_Quicktune::advance_stage(AP_Quicktune::axis_names axis)
{
//Move to next stage of tune
}

void AP_Quicktune::adjust_gain(AP_Quicktune::param_s param, float value)
{
//Change a gain.
//if limit is true, also do limit_gain() here - don't reduce by more than 100?
}

void AP_Quicktune::adjust_gain_limited(AP_Quicktune::param_s param, float value)
{
//Call adjust_gain after limiting.
}

float AP_Quicktune::get_gain_mul()
{
   return exp(log(2.0)/(UPDATE_RATE_HZ*double_time));
}

void AP_Quicktune::restore_all_params()
{

}

void AP_Quicktune::save_all_params()
{

}

bool AP_Quicktune::reached_limit()
{

}

void AP_Quicktune::get_all_params()
{

}

bool AP_Quicktune::item_in_bitmask(uint8_t item, uint32_t bitmask)
{
    if ((1<<item) & bitmask){
        return true;
    }
    return false;
}

bool AP_Quicktune::axis_done(AP_Quicktune::axis_names axis)
{
    return item_in_bitmask(uint8_t(axis), axes_done);
}

bool AP_Quicktune::axis_enabled(uint8_t axis)
{
    //Check whether axis has been enabled to be tuned.
    return item_in_bitmask(uint8_t(axis), axes_enabled);
}

bool AP_Quicktune::filter_done(AP_Quicktune::axis_names axis)
{
    //Check whether axis has been enabled to be tuned.
    return item_in_bitmask(uint8_t(axis), filters_done);
}

AP_Quicktune::param_s AP_Quicktune::get_pname(AP_Quicktune::axis_names axis, AP_Quicktune::stages stage)
{
    switch (axis)
    {
        case axis_names::RLL:
            if (stage == stages::P){
                return param_s::RLL_P;
            } return param_s::RLL_D;
        case axis_names::PIT:
            if (stage == stages::P){
                return param_s::RLL_P;
            } return param_s::RLL_D;
        case axis_names::YAW:
            if (stage == stages::P){
                return param_s::RLL_P;
            } return param_s::RLL_D;
        default:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }
}

float AP_Quicktune::get_param(AP_Quicktune::param_s param)
{
    switch (param)
    {
        case param_s::RLL_P:
            return attitude_control.get_rate_roll_pid().kP();
        case param_s::RLL_I:
            return attitude_control.get_rate_roll_pid().kI();
        case param_s::RLL_D:
            return attitude_control.get_rate_roll_pid().kD();
        case param_s::PIT_P:
            return attitude_control.get_rate_pitch_pid().kP();
        case param_s::PIT_I:
            return attitude_control.get_rate_pitch_pid().kI();
        case param_s::PIT_D:
            return attitude_control.get_rate_pitch_pid().kD();
        case param_s::YAW_P:
            return attitude_control.get_rate_yaw_pid().kP();
        case param_s::YAW_I:
            return attitude_control.get_rate_yaw_pid().kI();
        case param_s::YAW_D:
            return attitude_control.get_rate_yaw_pid().kD();
        default:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }
}

void AP_Quicktune::set_param(AP_Quicktune::param_s param, float value)
{
    switch (param)
    {
        case param_s::RLL_P:
            return attitude_control.get_rate_roll_pid().kP(value);
        case param_s::RLL_I:
            return attitude_control.get_rate_roll_pid().kI(value);
        case param_s::RLL_D:
            return attitude_control.get_rate_roll_pid().kD(value);
        case param_s::PIT_P:
            return attitude_control.get_rate_pitch_pid().kP(value);
        case param_s::PIT_I:
            return attitude_control.get_rate_pitch_pid().kI(value);
        case param_s::PIT_D:
            return attitude_control.get_rate_pitch_pid().kD(value);
        case param_s::YAW_P:
            return attitude_control.get_rate_yaw_pid().kP(value);
        case param_s::YAW_I:
            return attitude_control.get_rate_yaw_pid().kI(value);
        case param_s::YAW_D:
            return attitude_control.get_rate_yaw_pid().kD(value);
        default:
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }
}

AP_Quicktune::axis_names AP_Quicktune::get_axis(AP_Quicktune::param_s param)
{
    if (param < param_s::PIT_P){
        return axis_names::RLL;
    } else if (param < param_s::YAW_P){
        return axis_names::PIT;
    } else if (param < param_s::END){
        return axis_names::YAW;
    } else {
        return axis_names::END;
    }
}

float AP_Quicktune::gain_limit(AP_Quicktune::param_s param)
{
    if (get_axis(param) == axis_names::YAW){
        if (param == param_s::YAW_P){
            return yaw_p_max;
        }
        if (param == param_s::YAW_D){
            return yaw_d_max;
        }
    }
   return 0.0;
}





#endif //QUICKTUNE_ENABLED
