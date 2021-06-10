#include "Blimp.h"
/*
 * Init and run calls for loiter flight mode
 */


 bool ModeLoiter::init(bool ignore_checks){
    target_pos = blimp.position_ned;
    target_yaw = blimp.ahrs.get_yaw();
    loop_period = blimp.scheduler.get_loop_period_s();

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MIR init loiter: %f %f %f %f %f", target_pos.x, target_pos.y, target_pos.z, target_yaw, loop_period);
    return true;
 }

//Runs the main loiter controller
void ModeLoiter::run()
{
    float pilot_fwd = channel_front->get_control_in() / float(RC_SCALE) * g.max_xy_vel * loop_period;
    float pilot_rgt = channel_right->get_control_in() / float(RC_SCALE) * g.max_xy_vel * loop_period;
    float pilot_dwn = channel_down->get_control_in() / float(RC_SCALE) * g.max_xy_vel * loop_period;
    float pilot_yaw = channel_yaw->get_control_in()  / float(RC_SCALE) * g.max_yaw_vel * loop_period; 

    if (g.simple_mode == 0){
        //If simple mode is disabled, input is in body-frame, thus needs to be rotated.
        blimp.rotate_BF_to_NE(pilot_fwd, pilot_rgt);
    }
    target_pos.x += pilot_fwd;
    target_pos.y += pilot_rgt;
    target_pos.z += pilot_dwn;
    target_yaw = wrap_PI(target_yaw + pilot_yaw);

    //Pos controller's output becomes target for velocity controller
    Vector3f target_vel_ef{blimp.pid_pos_xy.update_all(target_pos, blimp.position_ned), 0};
    target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, blimp.position_ned.z);
    float yaw_ef = blimp.ahrs.get_yaw();
    float target_vel_yaw = blimp.pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef));
    blimp.pid_pos_yaw.set_target_rate(target_yaw);
    blimp.pid_pos_yaw.set_actual_rate(yaw_ef);

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -g.max_xy_vel, g.max_xy_vel),
                              constrain_float(target_vel_ef.y, -g.max_xy_vel, g.max_xy_vel),
                              constrain_float(target_vel_ef.z, -g.max_xy_vel, g.max_xy_vel)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -g.max_yaw_vel, g.max_yaw_vel);                                

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel_ef_c, blimp.velocity_ned_filt);
    float act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z, blimp.velocity_ned_filt.z);
    blimp.rotate_NE_to_BF(actuator.x, actuator.y);
    float act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c, blimp.vel_yaw_filt);

    if(!(blimp.g.dis_mask & (1<<(2-1)))){
    motors->front_out = actuator.x;
    } if(!(blimp.g.dis_mask & (1<<(1-1)))){
    motors->right_out = actuator.y;
    } if(!(blimp.g.dis_mask & (1<<(3-1)))){
    motors->down_out = act_down;
    } if(!(blimp.g.dis_mask & (1<<(4-1)))){
    motors->yaw_out  = act_yaw;
    }

    AP::logger().Write_PSC(target_pos*100.0f, blimp.position_ned*100.0f, target_vel_ef_c*100.0f, blimp.velocity_ned_filt*100.0f, blimp.velocity_ned*100.0f, target_yaw*100.0f, yaw_ef*100.0f); //last entries here are just for debugging
    AP::logger().Write_PSCZ(target_pos.z*100.0f, blimp.position_ned.z*100.0f, blimp.scheduler.get_loop_period_s()*100.0f, target_vel_ef_c.z*100.0f, blimp.velocity_ned_filt.z*100.0f, 0.0f, blimp.velocity_ned.z*100.0f, blimp.vel_yaw*100.0f, blimp.vel_yaw_filt*100.0f);
}
