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
    float target_vel_yaw = channel_yaw->get_control_in() / float(RC_SCALE) * g.max_yaw_vel; //TODO - consider separating this out

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel, blimp.velocity_ned_filt);
    blimp.rotate_NE_to_BF(actuator.x, actuator.y);
    float act_down = blimp.pid_vel_z.update_all(target_vel.z, blimp.velocity_ned_filt.z);
    float act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw, blimp.vel_yaw_filt);


    if(!(blimp.g.dis_mask & (1<<(2-1)))){
    motors->front_out = actuator.x;
    } if(!(blimp.g.dis_mask & (1<<(1-1)))){
    motors->right_out = actuator.y;
    } if(!(blimp.g.dis_mask & (1<<(3-1)))){
    motors->down_out = act_down;
    } if(!(blimp.g.dis_mask & (1<<(4-1)))){
    motors->yaw_out = act_yaw;
    }

    AP::logger().Write_PSC({0,0,0}, blimp.position_ned, target_vel*100.0f, blimp.velocity_ned_filt*100.0f, blimp.velocity_ned*100.0f, 0, 0);
    AP::logger().Write_PSCZ(0.0f, 0.0f, 0.0f, target_vel.z*100.0f, blimp.velocity_ned_filt.z*100.0f, 0.0f, blimp.velocity_ned.z*100.0f, blimp.vel_yaw*100.0f, blimp.vel_yaw_filt*100.0f);
}
