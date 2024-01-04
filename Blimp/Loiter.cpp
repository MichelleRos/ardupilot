#include "Blimp.h"

#include <AC_AttitudeControl/AC_PosControl.h>

#define MA 0.99
#define MO (1-MA)

void Loiter::run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    float scaler_x_n = 1;
    float scaler_y_n = 1;
    float scaler_z_n = 1;
    float scaler_yaw_n = 1;
    //IF FINNED BLIMP:
    // float xz_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->down_out);
    // if (xz_out > 1) {
    //     scaler_x_n = 1 / xz_out;
    //     scaler_z_n = 1 / xz_out;
    // }
    // scaler_x = scaler_x*MA + scaler_x_n*MO;
    // scaler_z = scaler_z*MA + scaler_z_n*MO;
    // float yyaw_out = fabsf(blimp.motors->right_out) + fabsf(blimp.motors->yaw_out);
    // if (yyaw_out > 1) {
    //     scaler_y_n = 1 / yyaw_out;
    //     scaler_yaw_n = 1 / yyaw_out;
    // }
    // scaler_y = scaler_y*MA + scaler_y_n*MO;
    // scaler_yaw = scaler_yaw*MA + scaler_yaw_n*MO;

    //IF PROP BLIMP:
    float xyaw_out = fabsf(blimp.motors->front_out) + fabsf(blimp.motors->yaw_out);
    if (xyaw_out > 1) {
        scaler_x_n = 1 / xyaw_out;
        scaler_yaw_n = 1 / xyaw_out;
    }
    scaler_x = scaler_x*MA + scaler_x_n*MO;
    scaler_yaw = scaler_yaw*MA + scaler_yaw_n*MO;

#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("BSC", "TimeUS,x,y,z,yaw,xn,yn,zn,yawn",
                                "Qffffffff",
                                AP_HAL::micros64(),
                                scaler_x, scaler_y, scaler_z, scaler_yaw, scaler_x_n, scaler_y_n, scaler_z_n, scaler_yaw_n);
#endif

    float yaw_ef = blimp.ahrs.get_yaw();
    Vector3f err_xyz = target_pos - blimp.pos_ned;
    float err_yaw = wrap_PI(target_yaw - yaw_ef);

    Vector4b zero;
    if ((fabsf(err_xyz.x) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.x = true;
    }
    if ((fabsf(err_xyz.y) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.y = true;
    }
    if ((fabsf(err_xyz.z) < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if ((fabsf(err_yaw)   < blimp.g.pid_dz) || !blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }

    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    Vector3f target_vel_ef;
    if (!axes_disabled.x)
        target_vel_ef.x = blimp.pid_pos_x.update_all(target_pos.x, blimp.pos_ned.x, dt, limit.x);
    if (!axes_disabled.x)
        target_vel_ef.y = blimp.pid_pos_y.update_all(target_pos.y, blimp.pos_ned.y, dt, limit.y);
    if (!axes_disabled.z) {
        target_vel_ef.z = blimp.pid_pos_z.update_all(target_pos.z, blimp.pos_ned.z, dt, limit.z);
    }

    float target_vel_yaw = 0;
    if (!axes_disabled.yaw) {
        target_vel_yaw = blimp.pid_pos_yaw.update_error(wrap_PI(target_yaw - yaw_ef), dt, limit.yaw);
        blimp.pid_pos_yaw.set_target_rate(target_yaw);
        blimp.pid_pos_yaw.set_actual_rate(yaw_ef);
    }

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_x, blimp.g.max_vel_x),
                             constrain_float(target_vel_ef.y, -blimp.g.max_vel_y, blimp.g.max_vel_y),
                             constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    Vector2f actuator;
    if (!axes_disabled.x) {
        actuator.x = blimp.pid_vel_x.update_all(target_vel_ef_c.x * scaler_x, blimp.vel_ned_filtd.x * scaler_x, dt, limit.x);
    }
    if (!axes_disabled.y) {
        actuator.y = blimp.pid_vel_y.update_all(target_vel_ef_c.y * scaler_y, blimp.vel_ned_filtd.y * scaler_y, dt, limit.y);
    }
    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z * scaler_z, blimp.vel_ned_filtd.z * scaler_z, dt, limit.z);
    }
    blimp.rotate_NE_to_BF(actuator);
    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yaw, blimp.vel_yaw_filtd * scaler_yaw, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        blimp.pid_pos_x.set_integrator(0);
        blimp.pid_pos_y.set_integrator(0);
        blimp.pid_pos_z.set_integrator(0);
        blimp.pid_pos_yaw.set_integrator(0);
        blimp.pid_vel_x.set_integrator(0);
        blimp.pid_vel_y.set_integrator(0);
        blimp.pid_vel_z.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
        target_pos = blimp.pos_ned;
        target_yaw = blimp.ahrs.get_yaw();
    }

    if (zero.x) {
        blimp.motors->front_out = 0;
    } else if (axes_disabled.x);
    else {
        blimp.motors->front_out = actuator.x;
    }
    if (zero.y) {
        blimp.motors->right_out = 0;
    } else if (axes_disabled.y);
    else {
        blimp.motors->right_out = actuator.y;
    }
    if (zero.z) {
        blimp.motors->down_out = 0;
    } else if (axes_disabled.z);
    else {
        blimp.motors->down_out = act_down;
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;
    } else if (axes_disabled.yaw);
    else {
        blimp.motors->yaw_out = act_yaw;
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(target_pos.x * 100.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(target_pos.y * 100.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(-target_pos.z * 100.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}

void Loiter::run_vel(Vector3f& target_vel_ef, float& target_vel_yaw, Vector4b axes_disabled)
{
    const float dt = blimp.scheduler.get_last_loop_time_s();

    Vector4b zero;
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(2-1)))) {
        zero.x = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(1-1)))) {
        zero.y = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(3-1)))) {
        zero.z = true;
    }
    if (!blimp.motors->_armed || (blimp.g.dis_mask & (1<<(4-1)))) {
        zero.yaw = true;
    }
    //Disabled means "don't update PIDs or output anything at all". Zero means actually output zero thrust. I term is limited in either case."
    Vector4b limit = zero || axes_disabled;

    Vector3f target_vel_ef_c{constrain_float(target_vel_ef.x, -blimp.g.max_vel_x, blimp.g.max_vel_x),
                             constrain_float(target_vel_ef.y, -blimp.g.max_vel_y, blimp.g.max_vel_y),
                             constrain_float(target_vel_ef.z, -blimp.g.max_vel_z, blimp.g.max_vel_z)};
    float target_vel_yaw_c = constrain_float(target_vel_yaw, -blimp.g.max_vel_yaw, blimp.g.max_vel_yaw);

    Vector2f actuator;
    if (!axes_disabled.x) {
        actuator.x = blimp.pid_vel_x.update_all(target_vel_ef_c.x * scaler_x, blimp.vel_ned_filtd.x * scaler_x, dt, limit.x);
    }

    if (!axes_disabled.y) {
        actuator.y = blimp.pid_vel_y.update_all(target_vel_ef_c.y * scaler_y, blimp.vel_ned_filtd.y * scaler_y, dt, limit.y);
    }

    float act_down = 0;
    if (!axes_disabled.z) {
        act_down = blimp.pid_vel_z.update_all(target_vel_ef_c.z * scaler_z, blimp.vel_ned_filtd.z * scaler_z, dt, limit.z);
    }

    float act_yaw = 0;
    if (!axes_disabled.yaw) {
        act_yaw = blimp.pid_vel_yaw.update_all(target_vel_yaw_c * scaler_yaw, blimp.vel_yaw_filtd * scaler_yaw, dt, limit.yaw);
    }

    if (!blimp.motors->armed()) {
        blimp.pid_vel_x.set_integrator(0);
        blimp.pid_vel_y.set_integrator(0);
        blimp.pid_vel_z.set_integrator(0);
        blimp.pid_vel_yaw.set_integrator(0);
    }

    blimp.rotate_NE_to_BF(actuator);

    if (zero.x) {
        blimp.motors->front_out = 0;
    } else if (axes_disabled.x);
    else {
        blimp.motors->front_out = actuator.x;
    }
    if (zero.y) {
        blimp.motors->right_out = 0;
    } else if (axes_disabled.y);
    else {
        blimp.motors->right_out = actuator.y;
    }
    if (zero.z) {
        blimp.motors->down_out = 0;
    } else if (axes_disabled.z);
    else {
        blimp.motors->down_out = act_down;
    }
    if (zero.yaw) {
        blimp.motors->yaw_out  = 0;
    } else if (axes_disabled.yaw);
    else {
        blimp.motors->yaw_out = act_yaw;
    }

#if HAL_LOGGING_ENABLED
    AC_PosControl::Write_PSCN(0.0, blimp.pos_ned.x * 100.0, 0.0, target_vel_ef_c.x * 100.0, blimp.vel_ned_filtd.x * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCE(0.0, blimp.pos_ned.y * 100.0, 0.0, target_vel_ef_c.y * 100.0, blimp.vel_ned_filtd.y * 100.0, 0.0, 0.0, 0.0);
    AC_PosControl::Write_PSCD(0.0, -blimp.pos_ned.z * 100.0, 0.0, -target_vel_ef_c.z * 100.0, -blimp.vel_ned_filtd.z * 100.0, 0.0, 0.0, 0.0);
#endif
}
