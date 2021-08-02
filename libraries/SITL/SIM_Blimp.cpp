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
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

Blimp::Blimp(const char *frame_str) :
    Aircraft(frame_str)
{
    fin = airfish;
    mass = 0.07;
    // frame_height = 0.0;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT; //Blimp does "land" when it gets to the ground.
    lock_step_scheduled = true;
}

// calculate rotational and linear accelerations
void Blimp::calculate_forces(const struct sitl_input &input, Vector3f &body_accel, Vector3f &rot_accel)
{
  // float fin_back  = filtered_servo_angle(input, 0);
  // float fin_front = filtered_servo_angle(input, 1);
  // float fin_right = filtered_servo_angle(input, 2);
  // float fin_left  = filtered_servo_angle(input, 3);

  // ::printf("FINS (%.1f %.1f %.1f %.1f)\n",
  //          fin_back, fin_front, fin_right, fin_left);

  float time = time_now_us*1e-6;

  //all fin setup
  for (uint8_t i=0; i<4; i++) {
    fin[i].last_angle = fin[i].angle;
    fin[i].last_time = fin[i].time;
    fin[i].angle = filtered_servo_angle(input, i)*75.0f; //for servo range of -75 deg to +75 deg
    fin[i].time = time;
    
    if (fin[i].angle < fin[i].last_angle) fin[i].dir = 0;
    else fin[i].dir = 1;
    
    fin[i].vel = (fin[i].angle - fin[i].last_angle)/(fin[i].time-fin[i].last_time); //deg/s
    fin[i].T = fin[i].vel^2 * K_Tan;
    fin[i].N = fin[i].vel^2 * K_Nor;
    if (fin[i].dir == 1) fin[i].N = -fin[1].N; //normal force flips when fin changes direction
    
    fin[i].Fx = 0;
    fin[i].Fy = 0;
    fin[i].Fz = 0;
  }

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
  fin[3].Fy = -fin[3].T*cos(fin[3].angle) - fin[3].N*sin(fin[3].angle);
  fin[3].Fz = -fin[3].T*sin(fin[3].angle) + fin[3].N*cos(fin[3].angle);

  //Temporary very simple/dodgy summing.
  Vector3f F_BF{0,0,0};
  for (uint8_t i=0; i<4; i++) {
    F_BF.x = F_BF.x + fin[i].Fx;
    F_BF.y = F_BF.y + fin[i].Fy;
    F_BF.z = F_BF.z + fin[i].Fz;
  }

  Vector3f body_accel{0,0,0};
  body_accel.x = F_BF.x/mass; //mass in kg, thus accel in m/s/s
  body_accel.y = F_BF.y/mass;
  body_accel.z = F_BF.z/mass;

  Vector3f rot_accel{0,0,0}; //rotational accel currently 0
// rot_accel += fin_torque *moment_of_inertia;
}

/*
  update the blimp simulation by one time step
 */
void Blimp::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel = Vector3f(0,0,0);
    //TODO Add "dcm.transposed() *  Vector3f(0, 0, calculate_buoyancy_acceleration());" for slight negative buoyancy.
    Vector3f body_accel = Vector3f(0,0,0);
    calculate_forces(input, fin*, body_accel, rot_accel);

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position(); //updates the position from the Vector3f pos
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

/*
SITL fns/vars
dcm.transposed()
velocity_ef
frame_time_us
voltage_scale
current

void Fin:calculate_fin_force(Fin fin, const struct sitl_input &input, Vector3f fin_force, Vector3f fin_torque){
  
  const float pwm = input.servos[servo];
  float command = pwm_to_command(pwm);
  float voltage_scale = voltage / voltage_max;


  // how much time has passed?
  float delta_time = frame_time_us * 1.0e-6f;
  //servo_speed = (prev servo pos - get curr servo pos) / time_diff
  //prev servo pos = get curr servo pos;

  Vector3f velocity_body = dcm.transposed() * velocity_ef; //velocity_ef is apparently given from SITL (see Rover sim)




  if (voltage_scale < 0.1) {
    // battery is dead
    rot_accel.zero();
    thrust.zero();
    current = 0;
    return;
  }
  
  
  //Vector3f thrust_ned


}
*/