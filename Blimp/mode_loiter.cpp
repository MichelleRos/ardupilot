#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */


 void ModeLoiter::init(bool ignore_checks){
    Vector3f pos_ef;
    bool gps_avail = blimp.ahrs.get_relative_position_NED_home(pos_ef);
        if (!gps_avail) {
        //Shouldn't reach this since it should failsafe into Land mode.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    Vector3f pos_bf = blimp.ahrs.get_rotation_body_to_ned().transposed() * pos_ef;

    target_pos.x = pos_bf.x;
    target_pos.y = pos_bf.y;
 }

//Runs the main loiter controller
void ModeLoiter::run()
{
    Vector3f pos_ef;
    bool gps_avail = blimp.ahrs.get_relative_position_NED_home(pos_ef);
        if (!gps_avail) {
        //Shouldn't reach this since it should failsafe into Land mode.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    Vector3f pos_bf = blimp.ahrs.get_rotation_body_to_ned().transposed() * pos_ef;

    //TODO Perhaps put a check in here to ensure that the target doesn't get too far from the vehicle.
    target_pos.x = target_pos.x + (channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_pos);
    target_pos.y = target_pos.y + (channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_pos);
    Vector3f target_pos3 = Vector3f(target_pos.x, target_pos.y, 0);

    //pos controller's output becomes target for velocity controller
    Vector3f target_vel =  Vector3f(blimp.pid_pos_xy.update_all(target_pos3, pos_bf), 0);

    Vector3f vel_ef;
    gps_avail = ahrs.get_velocity_NED(vel_ef); //earth-frame velocity
    if (!gps_avail) {
        //Shouldn't reach this since it should failsafe into Land mode.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    Vector3f vel_bf = ahrs.get_rotation_body_to_ned().transposed() * vel_ef;

    Vector3f vel_bf_xy = Vector3f(constrain_float(vel_bf.x, -g.max_xy_vel, g.max_xy_vel),
                                    constrain_float(vel_bf.y, -g.max_xy_vel, g.max_xy_vel),
                                    0);

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel, vel_bf_xy);

    motors->right_out = actuator.y;
    motors->front_out = actuator.x;

    //Currently yaw & down are simply disabled.
    motors->yaw_out = 0;
    motors->down_out = 0;

    AP::logger().Write_PSC(target_pos3, pos_bf, target_vel, vel_bf_xy, {0,0,0}, 0, 0);
    AP::logger().Write_PID(LOG_PIDN_MSG, blimp.pid_vel_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDE_MSG, blimp.pid_vel_xy.get_pid_info_y());
    AP::logger().Write_PID(LOG_PIDR_MSG, blimp.pid_pos_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDP_MSG, blimp.pid_pos_xy.get_pid_info_y());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(Fins::DesiredSpoolState::SHUT_DOWN);
    } else {
        motors->set_desired_spool_state(Fins::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
}