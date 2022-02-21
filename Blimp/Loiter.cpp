#include "Blimp.h"

#define MA 0.99
#define MO (1-MA)

void Loiter::run(Vector3f target_pos, float target_yaw, Vector4b axes_disabled){
    float scaler_xz_n;
    float xz_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->down_out);
    if (xz_out > 1) scaler_xz_n = 1 / xz_out;
    else scaler_xz_n = 1;
    scaler_xz = scaler_xz*MA + scaler_xz_n*MO;

    float scaler_yyaw_n;
    float yyaw_out = fabsf(blimp.motors->right_out) + fabsf(blimp.motors->yaw_out);
    if (yyaw_out > 1) scaler_yyaw_n = 1 / yyaw_out;
    else scaler_yyaw_n = 1;
    scaler_yyaw = scaler_yyaw*MA + scaler_yyaw_n*MO;

    if (AP_HAL::millis() % 1000 < 30) {
        send_BSC(scaler_xz, scaler_yyaw, scaler_xz_n, scaler_yyaw_n);
        gcs().send_named_float("TarX", target_pos.x);
        gcs().send_named_float("TarY", target_pos.y);
    }
    AP::logger().WriteStreaming("BSC", "TimeUS,xz,yyaw,xzn,yyawn",
                              "Qffff",
                              AP_HAL::micros64(),
                              scaler_xz, scaler_yyaw, scaler_xz_n, scaler_yyaw_n);

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

    Vector2f target_vel_ef_c_scaled_xy = {target_vel_ef_c.x * scaler_xz, target_vel_ef_c.y * scaler_yyaw};
    Vector2f vel_ned_filtd_scaled_xy = {blimp.vel_ned_filtd.x * scaler_xz, blimp.vel_ned_filtd.y * scaler_yyaw};

    Vector2f actuator = blimp.pid_vel_xy.update_all(target_vel_ef_c_scaled_xy, vel_ned_filtd_scaled_xy, {(float)limit.x, (float)limit.y});
    float act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z * scaler_xz, blimp.vel_ned_filtd.z * scaler_xz, limit.z);
    blimp.rotate_NE_to_BF(actuator);
    float act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yyaw, blimp.vel_yaw_filtd * scaler_yyaw, limit.yaw);

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


void Loiter::send_BSC(float s_xz, float s_yyaw, float s_xz_n, float s_yyaw_n){
    gcs().send_named_float("BSCxz", s_xz);
    gcs().send_named_float("BSCyyaw", s_yyaw);
    gcs().send_named_float("BSCxz_n", s_xz_n);
    gcs().send_named_float("BSCyyaw_n", s_yyaw_n);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scaler xz = %0.1f Scaler yyaw = %0.1f", s_xz, s_yyaw);
}