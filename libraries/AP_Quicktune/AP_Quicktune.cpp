#include "AP_Quicktune.h"

#if QUICKTUNE_ENABLED

const AP_Param::GroupInfo AP_Quicktune::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Quicktune enable
    // @Description: Enable quicktune system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_Quicktune, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: AXES
    // @DisplayName: Quicktune axes
    // @Description: Axes to tune
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw
    // @User: Standard
    AP_GROUPINFO("AXES", 2, AP_Quicktune, axes_enabled, 7),

    // @Param: DOUBLE_TIME
    // @DisplayName: Quicktune doubling time
    // @Description: Time to double a tuning parameter. Raise this for a slower tune.
    // @Range: 5 20
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("DOUBLE_TIME", 3, AP_Quicktune, double_time, 10),

    // @Param: GAIN_MARGIN
    // @DisplayName: Quicktune gain margin
    // @Description: Reduction in gain after oscillation detected. Raise this number to get a more conservative tune
    // @Range: 20 80
    // @Units: %
    // @User: Standard
    AP_GROUPINFO("GAIN_MARGIN", 4, AP_Quicktune, gain_margin, 60),

    // @Param: OSC_SMAX
    // @DisplayName: Quicktune oscillation rate threshold
    // @Description: Threshold for oscillation detection. A lower value will lead to a more conservative tune.
    // @Range: 1 10
    // @User: Standard
    AP_GROUPINFO("OSC_SMAX", 5, AP_Quicktune, osc_smax, 5),

    // @Param: YAW_P_MAX
    // @DisplayName: Quicktune Yaw P max
    // @Description: Maximum value for yaw P gain
    // @Range: 0.1 3
    // @User: Standard
    AP_GROUPINFO("YAW_P_MAX", 6, AP_Quicktune, yaw_p_max, 0.5),

    // @Param: YAW_D_MAX
    // @DisplayName: Quicktune Yaw D max
    // @Description: Maximum value for yaw D gain
    // @Range: 0.001 1
    // @User: Standard
    AP_GROUPINFO("YAW_D_MAX", 7, AP_Quicktune, yaw_d_max, 0.01),

    // @Param: RP_PI_RATIO
    // @DisplayName: Quicktune roll/pitch PI ratio
    // @Description: Ratio between P and I gains for roll and pitch. Raise this to get a lower I gain
    // @Range: 0.5 1.0
    // @User: Standard
    AP_GROUPINFO("RP_PI_RATIO", 8, AP_Quicktune, rp_pi_ratio, 1.0),

    // @Param: Y_PI_RATIO
    // @DisplayName: Quicktune Yaw PI ratio
    // @Description: Ratio between P and I gains for yaw. Raise this to get a lower I gain
    // @Range: 0.5 20
    // @User: Standard
    AP_GROUPINFO("Y_PI_RATIO", 9, AP_Quicktune, y_pi_ratio, 10),

    // @Param: AUTO_FILTER
    // @DisplayName: Quicktune auto filter enable
    // @Description: When enabled the PID filter settings are automatically set based on INS_GYRO_FILTER
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AUTO_FILTER", 10, AP_Quicktune, auto_filter, 1),

    // @Param: AUTO_SAVE
    // @DisplayName: Quicktune auto save
    // @Description: Number of seconds after completion of tune to auto-save. This is useful when using a 2 position switch for quicktune
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("AUTO_SAVE", 11, AP_Quicktune, auto_save, 0),

    // @Param: MAX_REDUCE
    // @DisplayName: Quicktune maximum gain reduction
    // @Description: This controls how much quicktune is allowed to lower gains from the original gains. If the vehicle already has a reasonable tune and is not oscillating then you can set this to zero to prevent gain reductions. The default of 20% is reasonable for most vehicles. Using a maximum gain reduction lowers the chance of an angle P oscillation happening if quicktune gets a false positive oscillation at a low gain, which can result in very low rate gains and a dangerous angle P oscillation.
    // @Units: %
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("MAX_REDUCE", 12, AP_Quicktune, max_reduce, 20),

    // @Param: OPTIONS
    // @DisplayName: Quicktune options
    // @Description: Additional options. When the Two Position Switch option is enabled then a high switch position will start the tune, low will disable the tune. you should also set a QUIK_AUTO_SAVE time so that you will be able to save the tune.
    // @Bitmask: 0:UseTwoPositionSwitch
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 13, AP_Quicktune, options, 0),

    AP_GROUPEND
};

// Call at loop rate
void AP_Quicktune::update(){

    if (enable < 1){
        return;
    }
    uint32_t now = AP_HAL::millis();

    if (now - last_update < UPDATE_PERIOD_MS) {
        return;
    }
    last_update = now;

    if (have_pilot_input()){
        last_pilot_input = now;
    }

    qt_switch_pos sw_pos_tune = qt_switch_pos::MID;
    qt_switch_pos sw_pos_save = qt_switch_pos::HIGH;
    if ((options & OPTIONS_TWO_POSITION) != 0){
        sw_pos_tune = qt_switch_pos::HIGH;
        sw_pos_save = qt_switch_pos::NONE;
    }

    const auto &vehicle = *AP::vehicle();

    if (sw_pos == sw_pos_tune && (!hal.util->get_soft_armed() || !vehicle.get_likely_flying()) && now > last_warning + 5000){
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Tuning: Must be flying to tune");
        last_warning = now;
        return;
    }
    if (sw_pos == qt_switch_pos::LOW || !hal.util->get_soft_armed() || !vehicle.get_likely_flying()){
        // Abort, revert parameters
        if (need_restore){
            need_restore = false;
            restore_all_params();
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Tuning: Reverted");
            tune_done_time = 0;
        }
        reset_axes_done();
        return;
    }
    if (sw_pos == sw_pos_save){
        // Save all params
        if (need_restore){
            need_restore = false;
            save_all_params();
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: Saved");
        }
    }
    if (sw_pos != sw_pos_tune){
        return;
    }

    if (now - last_stage_change < STAGE_DELAY){
        // Update slew gain
        if (slew_parm != param_s::END){
            float P = get_param_value(slew_parm);
            axis_names axis = get_axis(slew_parm);
            // local ax_stage = string.sub(slew_parm, -1)
            adjust_gain(slew_parm, P+slew_delta);
            slew_steps = slew_steps - 1;
            write_quik(get_slew_rate(axis), P, slew_parm);
            if (slew_steps == 0){
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f", get_param_name(slew_parm), P);
                slew_parm = param_s::END;
                if (get_current_axis() == axis_names::DONE){
                    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: DONE");
                    tune_done_time = now;
                }
            }
        }
        return;
    }

    axis_names axis = get_current_axis();

    if (axis == axis_names::DONE){
        // Nothing left to do, check autosave time
        if (tune_done_time != 0 && auto_save > 0){
            if (now - tune_done_time > (auto_save*1000)){
                need_restore = false;
                save_all_params();
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: Saved");
                tune_done_time = 0;
            }
        }
        return;
    }

    if (!need_restore){
        need_restore = true;
        // We are just starting tuning, get current values
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: Starting tune");
        // Get all params
        for (int8_t pname = 0; pname < uint8_t(param_s::END); pname++){
            param_saved[pname] = get_param_value(param_s(pname));
        }
        // Set up SMAX
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

    if (now - last_pilot_input < PILOT_INPUT_DELAY){
        return;
    }

    if (!BIT_IS_SET(filters_done, uint8_t(axis))){
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Starting %s tune", get_axis_name(axis));
        setup_filters(axis);
    }

    param_s pname = get_pname(axis, current_stage);
    float P = get_param_value(pname);
    float limit = gain_limit(pname);
    bool limited = (limit > 0.0 && P >= limit);
    float srate = get_slew_rate(axis);
    bool oscillating = srate > osc_smax;
    
    // Check if reached limit
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
            // We are lowering a D gain from the original gain. Also lower the P gain by the same amount so that we don't trigger P oscillation. We don't drop P by more than a factor of 2
            float ratio = fmaxf(new_gain / old_gain, 0.5);
            param_s P_name = param_s(uint8_t(pname)-2); //from D to P
            float old_P = get_param_value(P_name);;
            float new_P = old_P * ratio;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Adjusting %s %.3f -> %.3f", get_param_name(P_name), old_P, new_P);
            adjust_gain_limited(P_name, new_P);
        }
        // Set up slew gain
        slew_parm = pname;
        slew_target = limit_gain(pname, new_gain);
        slew_steps = UPDATE_RATE_HZ/2;
        slew_delta = (slew_target - get_param_value(pname)) / slew_steps;

        write_quik(srate, P, pname);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Tuning: %s done", get_param_name(pname));
        advance_stage(axis);
        last_stage_change = now;
    } else {
        float new_gain = P*get_gain_mul();
        if (new_gain <= 0.0001){
            new_gain = 0.001;
        }
        adjust_gain_limited(pname, new_gain);
        write_quik(srate, P, pname);
        if (now - last_gain_report > 3000){
            last_gain_report = now;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %.4f sr:%.2f", get_param_name(pname), new_gain, srate);
        }
    }
}

void AP_Quicktune::update_switch_pos(const  RC_Channel::AuxSwitchPos ch_flag) 
{
    sw_pos = qt_switch_pos(ch_flag);
}

void AP_Quicktune::reset_axes_done()
{
    axes_done = 0;
    filters_done = 0;
    current_stage = stages::D;
}

void AP_Quicktune::setup_filters(AP_Quicktune::axis_names axis)
{
    if (auto_filter <= 0){
        BIT_SET(filters_done, uint8_t(axis));
    }
    AP_InertialSensor *imu = AP_InertialSensor::get_singleton();
    if (imu == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "Quicktune: can't find IMU.");
        return;
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
    BIT_SET(filters_done, uint8_t(axis));
}

// Check for pilot input to pause tune
bool AP_Quicktune::have_pilot_input()
{
    const auto &rcmap = *AP::rcmap();
    float roll = rc().rc_channel(rcmap.roll()-1)->norm_input_dz();
    float pitch = rc().rc_channel(rcmap.pitch()-1)->norm_input_dz();
    float yaw = rc().rc_channel(rcmap.yaw()-1)->norm_input_dz();

    if (fabsf(roll) > 0 || fabsf(pitch) > 0 || fabsf(yaw) > 0){
        return true;
    }
    return false;
}

// Get the axis name we are working on, or DONE for all done 
AP_Quicktune::axis_names AP_Quicktune::get_current_axis()
{
    for (int8_t i = 0; i < int8_t(axis_names::DONE); i++){
        if (BIT_IS_SET(axes_enabled, i) == true && BIT_IS_SET(axes_done, i) == false){
            return axis_names(i);
        }
    }
    return axis_names::DONE;
}

float AP_Quicktune::get_slew_rate(AP_Quicktune::axis_names axis)
{
    switch(axis) {
    case axis_names::RLL:
        return attitude_control.get_rate_roll_pid().get_pid_info().slew_rate;
    case axis_names::PIT:
        return attitude_control.get_rate_pitch_pid().get_pid_info().slew_rate;
    case axis_names::YAW:
        return attitude_control.get_rate_yaw_pid().get_pid_info().slew_rate;
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
        BIT_SET(axes_done, uint8_t(axis));
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Tuning: %s done", get_axis_name(axis));
        current_stage = stages::D;
    }
}

void AP_Quicktune::adjust_gain(AP_Quicktune::param_s param, float value)
{
    need_restore = true;
    BIT_SET(param_changed, uint8_t(param));
    set_param_value(param, value);

    if (get_stage(param) == stages::P){
        // Also change I gain
        param_s iname = param_s(uint8_t(param)+1);
        // param_s ffname = param_s(uint8_t(param)+7);
        // float FF = get_param_value(ffname);
        // if (FF > 0){
        //     // If we have any FF on an axis then we don't couple I to P,
        //     // usually we want I = FF for a one second time constant for trim
        //     return;
        // }
        BIT_SET(param_changed, uint8_t(iname));

        // Work out ratio of P to I that we want
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
        // Check if we exceeded gain reduction
        float reduction_pct = 100.0 * (saved_value - value) / saved_value;
        if (reduction_pct > max_reduce){
            float new_value = saved_value * (100 - max_reduce) * 0.01;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Limiting %s %.3f -> %.3f", get_param_name(param), value, new_value);
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
   return expf(logf(2.0)/(UPDATE_RATE_HZ*double_time));
}

void AP_Quicktune::restore_all_params()
{
    for (int8_t pname = 0; pname < uint8_t(param_s::END); pname++){
        if (BIT_IS_SET(param_changed, pname)){
            set_param_value(param_s(pname), param_saved[pname]);
            BIT_CLEAR(param_changed, pname);
        }
    }
}

void AP_Quicktune::save_all_params()
{
    // for pname in pairs(params) do
    for (int8_t pname = 0; pname < uint8_t(param_s::END); pname++){
        if (BIT_IS_SET(param_changed, pname)){
            set_and_save_param_value(param_s(pname), get_param_value(param_s(pname)));
            param_saved[pname] = get_param_value(param_s(pname));
            BIT_CLEAR(param_changed, pname);
        }
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

AP_Float *AP_Quicktune::get_param_pointer(AP_Quicktune::param_s param)
{
    switch (param)
    {
        case param_s::RLL_P:
            return &attitude_control.get_rate_roll_pid().kP();
        case param_s::RLL_I:
            return &attitude_control.get_rate_roll_pid().kI();
        case param_s::RLL_D:
            return &attitude_control.get_rate_roll_pid().kD();
        case param_s::RLL_SMAX:
            return &attitude_control.get_rate_roll_pid().slew_limit();
        case param_s::RLL_FLTT:
            return &attitude_control.get_rate_roll_pid().filt_T_hz();
        case param_s::RLL_FLTD:
            return &attitude_control.get_rate_roll_pid().filt_D_hz();
        case param_s::RLL_FLTE:
            return &attitude_control.get_rate_roll_pid().filt_E_hz();
        case param_s::RLL_FF:
            return &attitude_control.get_rate_roll_pid().ff();
        case param_s::PIT_P:
            return &attitude_control.get_rate_pitch_pid().kP();
        case param_s::PIT_I:
            return &attitude_control.get_rate_pitch_pid().kI();
        case param_s::PIT_D:
            return &attitude_control.get_rate_pitch_pid().kD();
        case param_s::PIT_SMAX:
            return &attitude_control.get_rate_pitch_pid().slew_limit();
        case param_s::PIT_FLTT:
            return &attitude_control.get_rate_pitch_pid().filt_T_hz();
        case param_s::PIT_FLTD:
            return &attitude_control.get_rate_pitch_pid().filt_D_hz();
        case param_s::PIT_FLTE:
            return &attitude_control.get_rate_pitch_pid().filt_E_hz();
        case param_s::PIT_FF:
            return &attitude_control.get_rate_pitch_pid().ff();
        case param_s::YAW_P:
            return &attitude_control.get_rate_yaw_pid().kP();
        case param_s::YAW_I:
            return &attitude_control.get_rate_yaw_pid().kI();
        case param_s::YAW_D:
            return &attitude_control.get_rate_yaw_pid().kD();
        case param_s::YAW_SMAX:
            return &attitude_control.get_rate_yaw_pid().slew_limit();    
        case param_s::YAW_FLTT:
            return &attitude_control.get_rate_yaw_pid().filt_T_hz();
        case param_s::YAW_FLTD:
            return &attitude_control.get_rate_yaw_pid().filt_D_hz();
        case param_s::YAW_FLTE:
            return &attitude_control.get_rate_yaw_pid().filt_E_hz();
        case param_s::YAW_FF:
            return &attitude_control.get_rate_roll_pid().ff();
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_param_pointer - param was %d", uint8_t(param));
            // INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            return nullptr;
    }
}

float AP_Quicktune::get_param_value(AP_Quicktune::param_s param){
    AP_Float *ptr = get_param_pointer(param);
    if (ptr != nullptr) {
        return ptr->get();
    }
    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - get_param_value - param was %d", uint8_t(param));
    return 0.0;
}

void AP_Quicktune::set_param_value(AP_Quicktune::param_s param, float value){
    AP_Float *ptr = get_param_pointer(param);
    if (ptr != nullptr) {
       ptr->set(value);
       return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - set_param_value - param was %d", uint8_t(param));
    return;
}

void AP_Quicktune::set_and_save_param_value(AP_Quicktune::param_s param, float value){
    AP_Float *ptr = get_param_pointer(param);
    if (ptr != nullptr) {
       ptr->set_and_save(value);
       return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "INTERNAL ERROR - set_param_value - param was %d", uint8_t(param));
    return;
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

void AP_Quicktune::write_quik(float srate, float gain, AP_Quicktune::param_s param)
{
#ifdef HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("QUIK","TimeUS,SRate,Gain,Param,ParamNo", "QffNI", AP_HAL::micros64(), srate, gain, get_param_name(param), int(param));
#endif
}

#endif //QUICKTUNE_ENABLED
