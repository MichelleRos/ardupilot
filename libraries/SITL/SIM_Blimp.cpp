/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Blimp simulator class
*/

#include "SIM_Blimp.h"

#include <stdio.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

Blimp::Blimp(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 0.07;
    radius = 0.25;
    moment_of_inertia = {0, 0, 0.004375}; //m*r^2 for hoop...

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT; //Blimp does "land" when it gets to the ground.
    lock_step_scheduled = true;

    ::printf("Starting Blimp model\n");
}

// calculate rotational and linear accelerations
void Blimp::calculate_forces(const struct sitl_input &input, Vector3f &body_acc, Vector3f &rot_accel)
{
  float delta_time = frame_time_us * 1.0e-6f;

  if (!hal.scheduler->is_system_initialized()) {
      return;
  }

  //all fin setup
  for (uint8_t i=0; i<4; i++) {
    fin[i].last_angle = fin[i].angle;
    if (input.servos[i] == 0) {
        fin[i].angle = 0;
    } else {
        fin[i].angle = filtered_servo_angle(input, i)*radians(75.0f); //for servo range of -75 deg to +75 deg
    }
    
    if (fin[i].angle < fin[i].last_angle) fin[i].dir = 0;
    else fin[i].dir = 1;
    
    fin[i].vel = degrees(fin[i].angle - fin[i].last_angle)/delta_time; //deg/s
    fin[i].vel = constrain_float(fin[i].vel, -450, 450);
    fin[i].T = pow(fin[i].vel,2) * K_Tan;
    fin[i].N = pow(fin[i].vel,2) * K_Nor;
    if (fin[i].dir == 1) fin[i].N = -fin[1].N; //normal force flips when fin changes direction
    
    fin[i].Fx = 0;
    fin[i].Fy = 0;
    fin[i].Fz = 0;
  }

  //::printf("FINS (%.1f %.1f %.1f %.1f)  ", fin[0].angle, fin[1].angle, fin[2].angle, fin[3].angle);

  //TODO: Double-check that the directions are correct/correspond to the actual joystick directions.
  //Back fin
  fin[0].Fx = -fin[0].T*cos(fin[0].angle) - fin[0].N*sin(fin[0].angle);
  fin[0].Fz = -fin[0].T*sin(fin[0].angle) + fin[0].N*cos(fin[0].angle);

  //Front fin
  fin[1].Fx = fin[1].T*cos(fin[1].angle) + fin[1].N*sin(fin[1].angle);
  fin[1].Fz = -fin[1].T*sin(fin[1].angle) + fin[1].N*cos(fin[1].angle);

  //Right fin
  fin[2].Fy = -fin[2].T*cos(fin[2].angle) - fin[2].N*sin(fin[2].angle);
  fin[2].Fx = -fin[2].T*sin(fin[2].angle) + fin[2].N*cos(fin[2].angle);

  //Left fin
  fin[3].Fy = fin[3].T*cos(fin[3].angle) + fin[3].N*sin(fin[3].angle);
  fin[3].Fx = fin[3].T*sin(fin[3].angle) - fin[3].N*cos(fin[3].angle);

  Vector3f F_BF{0,0,0};
  for (uint8_t i=0; i<4; i++) {
    F_BF.x = F_BF.x + fin[i].Fx;
    F_BF.y = F_BF.y + fin[i].Fy;
    F_BF.z = F_BF.z + fin[i].Fz;
  }

  body_acc.x = F_BF.x/mass; //mass in kg, thus accel in m/s/s
  body_acc.y = F_BF.y/mass;
  body_acc.z = F_BF.z/mass - GRAVITY_MSS; //temporarily adding it in BF instead of WF

  Vector3f rot_T{0,0,0};
  rot_T.z = fin[2].Fx*0.25 + fin[3].Fx*0.25; //in N*m (Torque = force * lever arm)

  //rot accel = torque / moment of inertia
  rot_accel.x = 0;
  rot_accel.y = 0;
  rot_accel.z = rot_T.z / moment_of_inertia.z;
}

/*
  update the blimp simulation by one time step
 */
void Blimp::update(const struct sitl_input &input)
{
  float delta_time = frame_time_us * 1.0e-6f;

  Vector3f rot_accel = Vector3f(0,0,0);
  //TODO Add "dcm.transposed() *  Vector3f(0, 0, calculate_buoyancy_acceleration());" for slight negative buoyancy.
  calculate_forces(input, accel_body, rot_accel);

  // update rotational rates in body frame
  gyro += rot_accel * delta_time;

  Vector3f drag_gyr = gyro * drag_gyr_constant;
  rot_accel -= drag_gyr;

  gyro.x = constrain_float(gyro.x, -radians(2000.0f), radians(2000.0f));
  gyro.y = constrain_float(gyro.y, -radians(2000.0f), radians(2000.0f));
  gyro.z = constrain_float(gyro.z, -radians(2000.0f), radians(2000.0f));

  // update attitude
  dcm.rotate(gyro * delta_time);
  dcm.normalize();

  Vector3f accel_earth = dcm * accel_body;

  accel_earth += Vector3f(0.0f, 0.0f, GRAVITY_MSS);

  // if (on_ground() && accel_earth.z > 0) {
  //   accel_earth.z = 0;
  //   if (disp_now()) ::printf("Holding.\n");
  // }

  accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

  Vector3f drag = velocity_ef * drag_constant;
  accel_earth -= drag;

  velocity_ef += accel_earth * delta_time;
  position += (velocity_ef * delta_time).todouble(); //update position vector

  // update_external_payload(input);

  update_position(); //updates the position from the Vector3f position
  time_advance();
  update_mag_field_bf();
}
