#include "Blimp.h"

void Loiter::run(Vector3f target_pos, float target_yaw, Vector4b axes_disabled){
    float yaw_ef = blimp.ahrs.get_yaw();
    Vector3f err_xyz = target_pos - blimp.pos_ned;
    float err_yaw = wrap_PI(target_yaw - yaw_ef);

    Vector4b zero;
    if((fabsf(err_xyz.x) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) zero.x = true;
    if((fabsf(err_xyz.y) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) zero.y = true;
    if((fabsf(err_xyz.z) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) zero.z = true;
    if((fabsf(err_yaw)   < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) zero.yaw = true;

    Vector4b limit = zero || axes_disabled;

    //Debug
    // if(limit.x) ::printf("Zeroed on x.");
    // if(limit.y) ::printf("Zeroed on y.");
    // if(limit.z) ::printf("Zeroed on z.");
    // if(limit.yaw) ::printf("Zeroed on yaw.");
    // if(limit.x || limit.y || limit.z || limit.yaw) ::printf("\n");

    Vector3f target_vel_ef{blimp.pid_pos_xy.update_all(target_pos, blimp.pos_ned, {(float)limit.x, (float)limit.y, (float)limit.z}), 0};
    target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, limit.z);

    float target_vel_yaw = blimp.pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), limit.yaw);
    blimp.pid_pos_yaw.set_target_rate(target_yaw);
    blimp.pid_pos_yaw.set_actual_rate(yaw_ef);

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                              constrain_float(target_vel_ef.y, -blimp.g.max_vel_xy, blimp.g.max_vel_xy),
                              constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel_ef_c, blimp.vel_ned_filtd, {(float)limit.x, (float)limit.y, (float)limit.z});
    float act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z, blimp.vel_ned_filtd.z, limit.z);
    blimp.rotate_NE_to_BF(actuator);
    float act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c, blimp.vel_yaw_filtd, limit.yaw);

    if(zero.x){
        blimp.motors->front_out = 0;
    } else if (axes_disabled.x);
    else {
        blimp.motors->front_out = actuator.x;
    }
    if(zero.y){
        blimp.motors->right_out = 0;
    } else if (axes_disabled.y);
    else {
        blimp.motors->right_out = actuator.y;
    }
    if(zero.z){
        blimp.motors->down_out = 0;
    } else if (axes_disabled.z);
    else {
        blimp.motors->down_out = act_down;
    }
    if(zero.yaw){
        blimp.motors->yaw_out  = 0;
    } else if (axes_disabled.yaw);
    else {
        blimp.motors->yaw_out = act_yaw;
    }

    // AP::logger().Write_PSC(target_pos*100.0f, blimp.pos_ned*100.0f, target_vel_ef_c*100.0f, blimp.vel_ned_filtd*100.0f, blimp.vel_ned*100.0f, target_yaw*100.0f, yaw_ef*100.0f); //last entries here are just for debugging
    // AP::logger().Write_PSCZ(target_pos.z*100.0f, blimp.pos_ned.z*100.0f, blimp.scheduler.get_loop_period_s()*100.0f, target_vel_ef_c.z*100.0f, blimp.vel_ned_filtd.z*100.0f, 0.0f, blimp.vel_ned.z*100.0f, blimp.vel_yaw*100.0f, blimp.vel_yaw_filtd*100.0f);
    AP::logger().Write_PSCN(target_pos.x * 100.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AP::logger().Write_PSCE(target_pos.y * 100.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AP::logger().Write_PSCD(-target_pos.z * 100.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
}
