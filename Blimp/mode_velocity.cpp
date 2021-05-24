#include "Blimp.h"
/*
 * Init and run calls for velocity flight mode
 */

// Runs the main velocity controller
void ModeVelocity::run()
{
    Vector3f target_vel;
    target_vel.x = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_vel;
    target_vel.y = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_vel;
    target_vel.z = channel_down->get_control_in()  / float(RC_SCALE) * g.max_xy_vel;
    float target_vel_yaw = channel_yaw->get_control_in() / float(RC_SCALE) * g.max_xy_vel; //TODO - consider separating this out

    Vector3f vel_ef;
    bool gps_avail = ahrs.get_velocity_NED(vel_ef); //earth-frame velocity
    if (!gps_avail) {
        //Shouldn't reach this since it should failsafe into Land mode.
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Error: No GPS.");
    }
    Vector3f vel_bf = ahrs.get_rotation_body_to_ned().transposed() * vel_ef; //body-frame velocity
    float vel_yaw = blimp.ahrs.get_yaw_rate_earth();

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel, vel_bf);
    float act_down = blimp.pid_vel_z.update_all(target_vel.z, vel_bf.z);
    float act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw, vel_yaw);

    if(!(blimp.g.dis_mask & (1<<(2-1)))){
    motors->front_out = actuator.x;
    ::printf("Using front out\n");
    } if(!(blimp.g.dis_mask & (1<<(1-1)))){
    motors->right_out = actuator.y;
    ::printf("Using right out\n");
    } if(!(blimp.g.dis_mask & (1<<(3-1)))){
    motors->down_out = act_down;
    ::printf("Using down out\n");
    } if(!(blimp.g.dis_mask & (1<<(4-1)))){
    motors->yaw_out = act_yaw;
    ::printf("Using yaw out\n");
    }

    AP::logger().Write_PSC({0,0,0}, inertial_nav.get_position()*0.01f, target_vel, vel_bf, {0,0,0}, 0, 0);
    AP::logger().Write_PID(LOG_PIDN_MSG, blimp.pid_vel_xy.get_pid_info_x());
    AP::logger().Write_PID(LOG_PIDE_MSG, blimp.pid_vel_xy.get_pid_info_y());
}
