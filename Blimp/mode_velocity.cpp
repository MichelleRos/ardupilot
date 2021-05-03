#include "Blimp.h"
/*
 * Init and run calls for velocity flight mode
 */

// Runs the main velocity controller
void ModeVelocity::run()
{
    float desired_vel_x = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_vel;
    float desired_vel_y = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_vel;

    Vector3f vel_ef;
    bool gps_avail = ahrs.get_velocity_NED(vel_ef); //earth-frame velocity
    if (!gps_avail) {
        //TODO: Change so that it doesn't just keep trying to fly on velocity controller without GPS (while flooding the GCS).
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    Vector3f vel_bf = ahrs.get_rotation_body_to_ned().transposed() * vel_ef; //body-frame velocity

    Vector3f target_vel = Vector3f(desired_vel_x, desired_vel_y, 0);

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel, vel_bf);

    motors->right_out = actuator.y;
    motors->front_out = actuator.x;

    //Currently yaw & down are simply disabled.
    motors->yaw_out = 0;
    motors->down_out = 0;

    AP::logger().Write_PSC({0,0,0}, inertial_nav.get_position()*0.01f, target_vel, vel_bf, {0,0,0}, 0, 0);
    AP::logger().Write_PID(LOG_PIDN_MSG, blimp.pid_vel_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDE_MSG, blimp.pid_vel_xy.get_pid_info_y());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(Fins::DesiredSpoolState::SHUT_DOWN);
    } else {
        motors->set_desired_spool_state(Fins::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
}
