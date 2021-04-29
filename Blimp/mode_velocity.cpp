#include "Blimp.h"
/*
 * Init and run calls for velocity flight mode
 */

// Runs the main velocity controller
// should be called at 100hz or more
void ModeVelocity::run()
{
    float desired_vel_x = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_vel;
    float desired_vel_y = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_vel;

    Vector3f vel_ef;
    bool gps_avail = ahrs.get_velocity_NED(vel_ef); //earth-frame velocity
    if (!gps_avail) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    Vector3f vel_bf = ahrs.get_rotation_body_to_ned().transposed() * vel_ef; //body-frame velocity

    Vector3f target_vel = Vector3f(desired_vel_x, desired_vel_y, 0);

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel, vel_bf);

    //blimp.pid_vel_xy.get_p() + blimp.pid_vel_xy.get_i() + blimp.pid_vel_xy.get_d() + blimp.pid_vel_xy.get_ff();

    motors->right_out = -actuator.y;
    motors->front_out = -actuator.x;

    //Currently yaw & down are simply disabled.
    motors->yaw_out = 0;
    motors->down_out = 0;

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(Fins::DesiredSpoolState::SHUT_DOWN);
    } else {
        motors->set_desired_spool_state(Fins::DesiredSpoolState::THROTTLE_UNLIMITED);
    }
}
