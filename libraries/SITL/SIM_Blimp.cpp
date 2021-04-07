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
    frame_height = 0.0;
    ground_behavior = GROUND_BEHAVIOR_NONE;
}

// calculate rotational and linear accelerations
void Blimp::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    // float fin_back  = filtered_servo_angle(input, 0);
    // float fin_front = filtered_servo_angle(input, 1);
    // float fin_right = filtered_servo_angle(input, 2);
    // float fin_left  = filtered_servo_angle(input, 3);

    // ::printf("FINS (%.1f %.1f %.1f %.1f)\n",
    //          fin_back, fin_front, fin_right, fin_left);
}

/*
  update the blimp simulation by one time step
 */
void Blimp::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    for (uint8_t i=0; i<num_fins; i++) {
      Vector3f fin_force, fin_torque;
      calculate_fin_force(fin[i], input, fin_force, fin_torque);
      rot_accel += fin_torque *moment_of_inertia;
      body_accel += fin_force / mass;
    }

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

class Fins
{
public:

  Fin():
    servo(_servo) // what servo output drives this motor
    {}

  Vector3f position;
  float offset;
  float angle;
  float velocity; //velocity of fin in the 


}

void Fin:calculate_fin_force(Fin fin, const struct sitl_input &input, Vector3f fin_force, Vector3f fin_torque){
  
  const float pwm = input.servos[servo];
  float command = pwm_to_command(pwm);
  float voltage_scale = voltage / voltage_max;

  if (voltage_scale < 0.1) {
    // battery is dead
    rot_accel.zero();
    thrust.zero();
    current = 0;
    return;
  }
  
  
  //Vector3f thrust_ned


}