#include "Blimp.h"
/*
 * Init and run calls for auto flight mode
 */

bool ModeAuto::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();

    return true;
}

//Runs the main loiter controller
void ModeAuto::run()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms >= uint(blimp.g.stream_rate)) {
        last_report_ms = now_ms;
        tag = true;
    } else tag = false;
    
    // start or update mission
    if (waiting_to_start) {
        // don't start the mission until we have an origin
        Location loc;
        if (blimp.ahrs.get_origin(loc)) {
            if (tag) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "M: Got location, starting mission.");
            // start/resume the mission (based on MIS_RESTART parameter)
            mission.start_or_resume();
            waiting_to_start = false;

            // initialise mission change check (ignore results)
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        } else if (tag) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "M: Waiting for location.");
    } else {
        if (tag) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "M: Running mission.");
        // check for mission changes
        if (mis_change_detector.check_for_mission_change()) {
            // if mission is running restart the current command if it is a waypoint or spline command
            if ((mission.state() == AP_Mission::MISSION_RUNNING)) {
                if (mission.restart_current_nav_cmd()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed, restarted command");
                } else {
                    // failed to restart mission for some reason
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Auto mission changed but failed to restart command");
                }
            } else if (tag) GCS_SEND_TEXT(MAV_SEVERITY_INFO, "M: Mission not running.");
        }

        mission.update();
    }
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}

Location ModeAuto::loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const
{
    Location ret(cmd.content.location);

    // use current lat, lon if zero
    if (ret.lat == 0 && ret.lng == 0) {
        ret.lat = default_loc.lat;
        ret.lng = default_loc.lng;
    }
    // use default altitude if not provided in cmd
    if (ret.alt == 0) {
        // set to default_loc's altitude but in command's alt frame
        // note that this may use the terrain database
        int32_t default_alt;
        if (default_loc.get_alt_cm(ret.get_alt_frame(), default_alt)) {
            ret.set_alt_cm(default_alt, ret.get_alt_frame());
        } else {
            // default to default_loc's altitude and frame
            ret.set_alt_cm(default_loc.alt, default_loc.get_alt_frame());
        }
    }
    return ret;
}

bool ModeAuto::start_command(const AP_Mission::Mission_Command& cmd)
{
    if (blimp.should_log(MASK_LOG_CMD)) {
        blimp.logger.Write_Mission_Cmd(mission, cmd);
    }

    switch(cmd.id) {

        // case MAV_CMD_NAV_VTOL_TAKEOFF:
        // case MAV_CMD_NAV_TAKEOFF:                   // 22
        //     do_takeoff(cmd);
        //     break;

        case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
            do_nav_wp(cmd);
            break;

        // case MAV_CMD_NAV_VTOL_LAND:
        // case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        //     do_land(cmd);
        //     break;
        
        default:
        // unable to use the command, allow the vehicle to try the next command
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Command not supported. Skipped.");
        return false;
    }
    return true;
}

bool ModeAuto::verify_command(const AP_Mission::Mission_Command& cmd)
{
    if (blimp.flightmode != &blimp.mode_auto) {
        return false;
    }

    bool cmd_complete = false;

    switch (cmd.id) {
        case MAV_CMD_NAV_WAYPOINT:
            cmd_complete = verify_nav_wp(cmd);
            break;
        default:
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Command not supported. Skipped.");
            //Return true so it keeps going.
            return true;
    }
    return cmd_complete;
}

void ModeAuto::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;
    // if we are not on the ground switch to loiter or land
    if (!blimp.ap.land_complete) {
            set_mode(Mode::Number::LAND, ModeReason::MISSION_END);
    } else {
        // if we've landed it's safe to disarm
        blimp.arming.disarm(AP_Arming::Method::MISSIONEXIT);
    }
}

void ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    Location default_loc = blimp.current_loc;
    
    // get waypoint's location from command and send to wp_nav
    const Location target_loc = loc_from_cmd(cmd, default_loc);
    
    Vector3f target_cm;
    if(target_loc.get_vector_from_origin_NEU(target_cm)){
        target_pos.x = target_cm.x * 0.01;
        target_pos.y = target_cm.y * 0.01;
        target_pos.z = target_cm.z * 0.01;
    } else {
        //Shouldn't get here before origin is set. 
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
}

bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // // check if we have reached the waypoint
    // if ( !copter.wp_nav->reached_wp_destination() ) {
    //     return false;
    // }

    // // start timer if necessary
    // if (loiter_time == 0) {
    //     loiter_time = millis();
    //     if (loiter_time_max > 0) {
    //         // play a tone
    //         AP_Notify::events.waypoint_complete = 1;
    //     }
    // }

    // // check if timer has run out
    // if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
    //     if (loiter_time_max == 0) {
    //         // play a tone
    //         AP_Notify::events.waypoint_complete = 1;
    //     }
    if (blimp.loiter->target_accepted()){
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Not at WP yet.");
    return false;
}