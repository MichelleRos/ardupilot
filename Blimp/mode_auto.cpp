#include "Blimp.h"
/*
 * Init and run calls for auto flight mode
 */

bool ModeAuto::init(bool ignore_checks)
{
    target_pos = blimp.pos_ned;
    target_yaw = blimp.ahrs.get_yaw();
    waiting_to_start = true;
    origin = target_pos;
    destination = target_pos;

    scurve_prev_leg.init();
    scurve_this_leg.init();
    scurve_next_leg.init();

    return true;
}

//Runs the main auto controller
void ModeAuto::run()
{
    // start or update mission
    if (waiting_to_start) {
        // don't start the mission until we have an origin
        Location loc;
        if (blimp.ahrs.get_origin(loc)) {
            // start/resume the mission (based on MIS_RESTART parameter)
            mission.start_or_resume();
            waiting_to_start = false;

            // initialise mission change check (ignore results)
            IGNORE_RETURN(mis_change_detector.check_for_mission_change());
        }
    } else {
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
            }
        }

        mission.update();
    }
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
    gcs().send_named_float("TarX", target_pos.x);
    gcs().send_named_float("TarY", target_pos.y);
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

Vector3f ModeAuto::vec_from_loc(const Location& loc)
{
    Vector3f vec;
    if(loc.get_vector_from_origin_NEU(vec)){
        vec.x = vec.x * 0.01;
        vec.y = vec.y * 0.01;
        vec.z = - vec.z * 0.01;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_EMERGENCY, "get_vector_from_origin_NEU() returned false.");
    }

    return vec;
}

Vector3f ModeAuto::vec_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc){
    Location loc = loc_from_cmd(cmd, default_loc);
    return vec_from_loc(loc);
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
    // switch to mode loiter
    set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
}

// get waypoint's location from command and send to scurves
// only called with each new nav wp command
void ModeAuto::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    Location default_loc = blimp.current_loc;

    origin = destination;
    destination = vec_from_cmd(cmd, default_loc);

    scurve_prev_leg = scurve_this_leg;
    scurve_this_leg.calculate_track(origin, destination,
                                g.wp_vel, loiter->max_vel_z, loiter->max_vel_z,
                                g.wp_accel, g.wp_accel,
                                g.wp_snap, g.wp_jerk);
    scurve_this_leg_origin = origin;
    AP_Mission::Mission_Command next_cmd;
    if (!mission.get_next_nav_cmd(cmd.index+1, next_cmd)) {
        fast_wp = false;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "MIR: Last leg.");
        return;
    }
    const Location dest_loc = loc_from_cmd(cmd, default_loc);
    Vector3f next_dest = vec_from_cmd(next_cmd, dest_loc);
    scurve_next_leg.calculate_track(destination, next_dest,
                                g.wp_vel, loiter->max_vel_z, loiter->max_vel_z,
                                g.wp_accel, g.wp_accel,
                                g.wp_snap, g.wp_jerk);
    fast_wp = true;
}

// Advances along the waypoint and returns whether or not it has reached the waypoint
// Called on every loop
bool ModeAuto::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();
    bool s_finished = false;
    if (blimp.loiter->target_within(g.wp_rad)){
        s_finished = scurve_this_leg.advance_target_along_track(scurve_prev_leg, scurve_next_leg, g.wp_rad, g.wp_accel, fast_wp, dt, scurve_this_leg_origin, target_vel, target_accel);
        target_pos = scurve_this_leg_origin;
        scurve_this_leg_origin = origin;
    }

    if (s_finished){
        return true;
    }
    return false;
}
