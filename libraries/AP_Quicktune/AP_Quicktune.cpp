#include "AP_Quicktune.h"

#if QUICKTUNE_ENABLED

AP_Quicktune *AP_Quicktune::singleton;

const AP_Param::GroupInfo AP_Quicktune::var_info[] = {
    // @Param: QUIK_ENABLE
    // @DisplayName: Enable Quicktune.
    // @Description:
    // @User: Standard
    AP_GROUPINFO("ENABLE", 0, AP_Quicktune, enable, 0),

    // @Param: QUIK_AXES
    // @DisplayName: Bitmask of axes to tune.
    // @Description:
    // @User: Standard
    AP_GROUPINFO("AXES", 1, AP_Quicktune, axes_enabled, 7),

    // @Param: QUIK_DOUBLE_TIME
    // @DisplayName: Double time.
    // @Description: Time taken to double the gain.
    // @User: Standard
    AP_GROUPINFO("DOUBLE_TIME", 2, AP_Quicktune, double_time, 10),

    // @Param: QUIK_GAIN_MARGIN
    // @DisplayName: Gain margin.
    // @Description:
    // @User: Standard
    AP_GROUPINFO("GAIN_MARGIN", 3, AP_Quicktune, gain_margin, 60),

    // @Param: QUIK_OSC_SMAX
    // @DisplayName: Slewrate for oscillation detection.
    // @Description:
    // @User: Standard
    AP_GROUPINFO("OSC_SMAX", 4, AP_Quicktune, osc_smax, 5),

    // @Param: QUIK_YAW_P_MAX
    // @DisplayName: Yaw P max.
    // @Description: Maximum Yaw P gain to tune to.
    // @User: Standard
    AP_GROUPINFO("YAW_P_MAX", 5, AP_Quicktune, yaw_p_max, 0.5),

    // @Param: QUIK_YAW_D_MAX
    // @DisplayName: Yaw D max.
    // @Description: Maximum Yaw D gain to tune to.
    // @User: Standard
    AP_GROUPINFO("YAW_D_MAX", 6, AP_Quicktune, yaw_d_max, 0.01),

    // @Param: QUIK_RP_PI_RATIO
    // @DisplayName: Ratio between P and I gains for roll and pitch axes.
    // @Description:
    // @User: Standard
    AP_GROUPINFO("RP_PI_RATIO", 7, AP_Quicktune, rp_pi_ratio, 1.0),

    // @Param: QUIK_Y_PI_RATIO
    // @DisplayName: Ratio between P and I gains for yaw axis.
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
    // @DisplayName: Maximum reduction in percent.
    // @Description:
    // @User: Standard
    AP_GROUPINFO("MAX_REDUCE", 12, AP_Quicktune, max_reduce, 20),

    // @Param: QUIK_OPTIONS
    // @DisplayName: Options bitmask.
    // @Description: Bit 1 uses 2 position switch.
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 13, AP_Quicktune, options, 0),

    AP_GROUPEND
};

// Call at loop rate
void AP_Quicktune::update(){

    if (enable < 1){
        return;
    }
    if (have_pilot_input()){
        last_pilot_input = get_time();
    }
    uint8_t sw_pos;
    bool sw_set = rc().get_aux_cached(RC_Channel::AUX_FUNC::QUICKTUNE, sw_pos);
    if (!sw_set){
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
        // abort, revert parameters
        if (need_restore){
            need_restore = false;
            restore_all_params();
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Tuning: reverted");
            tune_done_time = 0;
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
        //update_slew_gain(); ->inlined
        if (slew_parm != param_s::END){
            float P = get_param_value(slew_parm);
            // local axis = param_axis(slew_parm)
            // local ax_stage = string.sub(slew_parm, -1)
            adjust_gain(slew_parm, P+slew_delta);
            slew_steps = slew_steps - 1;
            // logger.write('QUIK','SRate,Gain,Param', 'ffn', get_slew_rate(axis), P:get(), axis .. ax_stage)
            if (slew_steps == 0){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f", get_param_name(slew_parm), P);
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
        if (!is_zero(tune_done_time) && auto_save > 0){
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
        // we are just starting tuning, get current values
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: starting tune");
        // get_all_params(); ->inlined
        for (int8_t pname = 0; pname < uint8_t(param_s::END); pname++){
            param_saved[pname] = get_param_value(param_s(pname));
        }
        setup_SMAX();
    }

    if (get_time() - last_pilot_input < PILOT_INPUT_DELAY){
        return;
    }

    if (!item_in_bitmask(uint8_t(axis), filters_done)){
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting %s tune", get_axis_name(axis));
        setup_filters(axis);
    }

    float srate = get_slew_rate(axis);
    param_s pname = get_pname(axis, current_stage);
    float P = get_param_value(pname);
    bool oscillating = srate > osc_smax;

    // float limited = reached_limit(pname, P); -> inlined
    float limit = gain_limit(pname);
    bool limited = (limit > 0.0 && P >= limit);
       
    if (limited || oscillating){
        float reduction = (100.0-gain_margin)*0.01;
        if (!oscillating){
            reduction = 1.0;
        }
        float new_gain = P * reduction;
        if (limit > 0.0 && new_gain > limit){
            new_gain = limit;
        }
        float old_gain = param_saved[uint8_t(pname)];
        if (new_gain < old_gain && (pname == param_s::PIT_D || pname == param_s::RLL_D)){
            //-- we are lowering a D gain from the original gain. Also lower the P gain by the same amount so that we don't trigger P oscillation. We don't drop P by more than a factor of 2
            float ratio = fmaxf(new_gain / old_gain, 0.5);
            param_s P_name = param_s(uint8_t(pname)-2); //from D to P
            float old_P = get_param_value(P_name);;
            float new_P = old_P * ratio;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "adjusting %s %.3f -> %.3f", get_param_name(P_name), old_P, new_P);
            adjust_gain_limited(P_name, new_P);
        }
        // setup_slew_gain(pname, new_gain); -> inlined
        slew_parm = pname;
        slew_target = limit_gain(pname, new_gain);
        slew_steps = UPDATE_RATE_HZ/2;
        slew_delta = (slew_target - get_param_value(pname)) / slew_steps;

        logger->WriteStreaming("QUIK","TimeUS,SRate,Gain,Param", "QffI", AP_HAL::micros64(), srate, P, int(pname));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Tuning: %s done", get_param_name(pname));
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
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f sr:%.2f", get_param_name(pname), new_gain, srate);
        }
    }
}

// Reset the parameter for which axes have been done.
void AP_Quicktune::reset_axes_done()
{
    axes_done = 0;
    filters_done = 0;
    current_stage = stages::D;
}

// Check each SMAX param, set to DEFAULT_SMAX if it is zero.
void AP_Quicktune::setup_SMAX()
{
    param_s is[3];
    is[0] = param_s::RLL_SMAX;
    is[1] = param_s::PIT_SMAX;
    is[2] = param_s::YAW_SMAX;

    for (uint8_t i = 0; i < 3; i++){
        float smax = get_param_value(is[i]);
        if (smax <= 0){
            adjust_gain(is[i], DEFAULT_SMAX); 
        }
    }
}

void AP_Quicktune::setup_filters(AP_Quicktune::axis_names axis)
{
    if (auto_filter <= 0){
        set_bitmask(true, filters_done, uint8_t(axis));
    }
    float gyro_filter = imu->get_gyro_filter_hz();
    adjust_gain(get_pname(axis, stages::FLTT), gyro_filter * FLTT_MUL);
    adjust_gain(get_pname(axis, stages::FLTD), gyro_filter * FLTT_MUL);

    if (axis == axis_names::YAW){
        float FLTE = get_param_value(param_s::YAW_FLTE);
        if (FLTE < 0.0 || FLTE > YAW_FLTE_MAX){
            adjust_gain(param_s::YAW_FLTE, YAW_FLTE_MAX);
        }
    }
    set_bitmask(true, filters_done, uint8_t(axis));
}

// Check for pilot input to pause tune
bool AP_Quicktune::have_pilot_input()
{
    float roll = rc().rc_channel(rcmap->roll()-1)->norm_input_dz();
    float pitch = rc().rc_channel(rcmap->pitch()-1)->norm_input_dz();
    float yaw = rc().rc_channel(rcmap->yaw()-1)->norm_input_dz();

    if (fabsf(roll) > 0 || fabsf(pitch) > 0 || fabsf(yaw) > 0){
        return true;
    }
    return false;
}

// Get the axis name we are working on, or DONE for all done 
AP_Quicktune::axis_names AP_Quicktune::get_current_axis()
{
    for (int8_t i = 0; i < int8_t(axis_names::DONE); i++){
        if (item_in_bitmask(i, axes_enabled) == true && item_in_bitmask(i, axes_done) == false){
            return axis_names(i);
        }
    }
    return axis_names::DONE;
}

float AP_Quicktune::get_slew_rate(AP_Quicktune::axis_names axis)
{
    switch(axis) {
    case axis_names::RLL:
        return attitude_control->get_rate_roll_pid().get_pid_info().slew_rate;
    case axis_names::PIT:
        return attitude_control->get_rate_pitch_pid().get_pid_info().slew_rate;
    case axis_names::YAW:
        return attitude_control->get_rate_yaw_pid().get_pid_info().slew_rate;
    default:
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_slew_rate - axis was %d", uint8_t(axis));
        return 0.0;
    }
}

// Move to next stage of tune
void AP_Quicktune::advance_stage(AP_Quicktune::axis_names axis)
{
    if (current_stage == stages::D){
        current_stage = stages::P;
    } else {
        set_bitmask(true, axes_done, uint8_t(axis));
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: %s done", get_axis_name(axis));
        current_stage = stages::D;
    }
}

void AP_Quicktune::adjust_gain(AP_Quicktune::param_s param, float value)
{
    need_restore = true;
    set_bitmask(true, param_changed, uint8_t(param));
    set_param_value(param, value);

    if (get_stage(param) == stages::P){
        // also change I gain
        param_s iname = param_s(uint8_t(param)+1);
        param_s ffname = param_s(uint8_t(param)+7);
        float FF = get_param_value(ffname);
        if (FF > 0){
            // if we have any FF on an axis then we don't couple I to P,
            // usually we want I = FF for a one second time constant for trim
            return;
        }
        set_bitmask(true, param_changed, uint8_t(iname));

        // work out ratio of P to I that we want
        float pi_ratio = rp_pi_ratio;
        if (get_axis(param) == axis_names::YAW){
            pi_ratio = y_pi_ratio;
        }
        if (pi_ratio >= 1){
            set_param_value(iname, value/pi_ratio);
        }
    }

}

void AP_Quicktune::adjust_gain_limited(AP_Quicktune::param_s param, float value)
{
    adjust_gain(param, limit_gain(param, value));
}

float AP_Quicktune::limit_gain(AP_Quicktune::param_s param, float value)
{
    float saved_value = param_saved[uint8_t(param)];
    if (max_reduce >= 0 && max_reduce < 100 && saved_value > 0){
        // check if we exceeded gain reduction
        float reduction_pct = 100.0 * (saved_value - value) / saved_value;
        if (reduction_pct > max_reduce){
            float new_value = saved_value * (100 - max_reduce) * 0.01;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "limiting %s %.3f -> %.3f", get_param_name(param), value, new_value);
            value = new_value;
        }
    }
   return value;
}

const char* AP_Quicktune::get_param_name(AP_Quicktune::param_s param)
{
    switch (param)
    {
        case param_s::RLL_P:
            return "Roll P";
        case param_s::RLL_I:
            return "Roll I";
        case param_s::RLL_D:
            return "Roll D";
        case param_s::PIT_P:
            return "Pitch P";
        case param_s::PIT_I:
            return "Pitch I";
        case param_s::PIT_D:
            return "Pitch D";
        case param_s::YAW_P:
            return "Yaw P";
        case param_s::YAW_I:
            return "Yaw I";
        case param_s::YAW_D:
            return "Yaw D";
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_param_name - param was %d", uint8_t(param));
            return "UNK";
    }
}

float AP_Quicktune::get_gain_mul()
{
   return exp(log(2.0)/(UPDATE_RATE_HZ*double_time));
}

void AP_Quicktune::restore_all_params()
{
    for (int8_t pname = 0; pname < uint8_t(param_s::END); pname++){
        if (item_in_bitmask(pname, param_changed)){
            set_param_value(param_s(pname), param_saved[pname]);
            set_bitmask(false, param_changed, pname);
        }
    }
}

void AP_Quicktune::save_all_params()
{
    // for pname in pairs(params) do
    for (int8_t pname = 0; pname < uint8_t(param_s::END); pname++){
        if (item_in_bitmask(pname, param_changed)){
            set_and_save_param_value(param_s(pname), get_param_value(param_s(pname)));
            param_saved[pname] = get_param_value(param_s(pname));
            set_bitmask(false, param_changed, pname);
        }
    }
}

bool AP_Quicktune::item_in_bitmask(uint8_t item, uint32_t bitmask)
{
    if ((1<<item) & bitmask){
        return true;
    }
    return false;
}

#define bitset(byte,nbit)   ((byte) |=  (1<<(nbit)))
#define bitclear(byte,nbit) ((byte) &= ~(1<<(nbit)))
void AP_Quicktune::set_bitmask(bool value, uint32_t &bitmask, uint8_t position)
{
    if(value){
        bitset(bitmask, position);
    } else {
        bitclear(bitmask, position);
    }
}

AP_Quicktune::param_s AP_Quicktune::get_pname(AP_Quicktune::axis_names axis, AP_Quicktune::stages stage)
{
    switch (axis)
    {
        case axis_names::RLL:
            switch (stage)
            {
                case stages::P:
                    return param_s::RLL_P;
                case stages::D:
                    return param_s::RLL_D;
                case stages::FLTT:
                    return param_s::RLL_FLTT;
                case stages::FLTD:
                    return param_s::RLL_FLTD;
                default:
                    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_pname - axis was %d, stage was %d", uint8_t(axis), uint8_t(stage));
                    return param_s::END;
            }
        case axis_names::PIT:
            switch (stage)
            {
                case stages::P:
                    return param_s::PIT_P;
                case stages::D:
                    return param_s::PIT_D;
                case stages::FLTT:
                    return param_s::PIT_FLTT;
                case stages::FLTD:
                    return param_s::PIT_FLTD;
                default:
                    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_pname - axis was %d, stage was %d", uint8_t(axis), uint8_t(stage));
                    return param_s::END;
            }
        case axis_names::YAW:
            switch (stage)
            {
                case stages::P:
                    return param_s::YAW_P;
                case stages::D:
                    return param_s::YAW_D;
                case stages::FLTT:
                    return param_s::YAW_FLTT;
                case stages::FLTD:
                    return param_s::YAW_FLTD;
                default:
                    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_pname - axis was %d, stage was %d", uint8_t(axis), uint8_t(stage));
                    return param_s::END;
            }
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_pname - axis was %d", uint8_t(axis));
            // INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return param_s::END;
    }
}

AP_Quicktune::stages AP_Quicktune::get_stage(AP_Quicktune::param_s param)
{
    if (param == param_s::RLL_P || param == param_s::PIT_P || param == param_s::YAW_P){
        return stages::SMAX;
    } else if (param == param_s::RLL_I || param == param_s::PIT_I || param == param_s::YAW_I){
        return stages::I;
    } else if (param == param_s::RLL_FF || param == param_s::PIT_FF || param == param_s::YAW_FF){
        return stages::FF;
    } else {
        return stages::END;
    }
}

float AP_Quicktune::get_param_value(AP_Quicktune::param_s param)
{
    switch (param)
    {
        case param_s::RLL_P:
            return attitude_control->get_rate_roll_pid().kP();
        case param_s::RLL_I:
            return attitude_control->get_rate_roll_pid().kI();
        case param_s::RLL_D:
            return attitude_control->get_rate_roll_pid().kD();
        case param_s::PIT_P:
            return attitude_control->get_rate_pitch_pid().kP();
        case param_s::PIT_I:
            return attitude_control->get_rate_pitch_pid().kI();
        case param_s::PIT_D:
            return attitude_control->get_rate_pitch_pid().kD();
        case param_s::YAW_P:
            return attitude_control->get_rate_yaw_pid().kP();
        case param_s::YAW_I:
            return attitude_control->get_rate_yaw_pid().kI();
        case param_s::YAW_D:
            return attitude_control->get_rate_yaw_pid().kD();
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_param_value - param was %d", uint8_t(param));
            // INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return 0.0;
            break;
    }
}

void AP_Quicktune::set_param_value(AP_Quicktune::param_s param, float value)
{
    switch (param)
    {
        case param_s::RLL_P:
            attitude_control->get_rate_roll_pid().kP(value);
            return;
        case param_s::RLL_I:
            attitude_control->get_rate_roll_pid().kI(value);
            return;
        case param_s::RLL_D:
            attitude_control->get_rate_roll_pid().kD(value);
            return;
        case param_s::PIT_P:
            attitude_control->get_rate_pitch_pid().kP(value);
            return;
        case param_s::PIT_I:
            attitude_control->get_rate_pitch_pid().kI(value);
            return;
        case param_s::PIT_D:
            attitude_control->get_rate_pitch_pid().kD(value);
            return;
        case param_s::YAW_P:
            attitude_control->get_rate_yaw_pid().kP(value);
            return;
        case param_s::YAW_I:
            attitude_control->get_rate_yaw_pid().kI(value);
            return;
        case param_s::YAW_D:
            attitude_control->get_rate_yaw_pid().kD(value);
            return;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - set param value - param was %d", uint8_t(param));
            // INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
    }
}

void AP_Quicktune::set_and_save_param_value(AP_Quicktune::param_s param, float value)
{
    switch (param)
    {
        case param_s::RLL_P:
            attitude_control->get_rate_roll_pid().kP_s(value);
            return;
        case param_s::RLL_I:
            attitude_control->get_rate_roll_pid().kI_s(value);
            return;
        case param_s::RLL_D:
            attitude_control->get_rate_roll_pid().kD_s(value);
            return;
        case param_s::PIT_P:
            attitude_control->get_rate_pitch_pid().kP_s(value);
            return;
        case param_s::PIT_I:
            attitude_control->get_rate_pitch_pid().kI_s(value);
            return;
        case param_s::PIT_D:
            attitude_control->get_rate_pitch_pid().kD_s(value);
            return;
        case param_s::YAW_P:
            attitude_control->get_rate_yaw_pid().kP_s(value);
            return;
        case param_s::YAW_I:
            attitude_control->get_rate_yaw_pid().kI_s(value);
            return;
        case param_s::YAW_D:
            attitude_control->get_rate_yaw_pid().kD_s(value);
            return;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - set_and_save_param_value - param was %d", uint8_t(param));
            // INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return;
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

const char* AP_Quicktune::get_axis_name(AP_Quicktune::axis_names axis)
{
    switch (axis)
    {
        case axis_names::RLL:
            return "Roll";
        case axis_names::PIT:
            return "Pitch";
        case axis_names::YAW:
            return "Yaw";
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_axis_name - axis was %d", uint8_t(axis));
            return "UNK";
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
