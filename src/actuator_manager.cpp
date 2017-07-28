/**
 * @file actuator_manager.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/actuator_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuator_manager::set_wheel_speeds(const argos::CVector2& c_heading) {
   /* Get the heading angle */
  argos::CRadians heading_angle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
  argos::Real heading_length = c_heading.Length();
   /* Clamp the speed so that it's not greater than max_speed */
  argos::Real base_angular_wheel_speed = argos::Min<argos::Real>(heading_length, mc_params.wheels.max_speed);

   /* Turning state switching conditions */
   if (Abs(heading_angle) <= mc_params.wheels.no_turn_threshold) {
      /* No Turn, heading angle very small */
      m_turning_state = turning_state::NO_TURN;
   } else if (Abs(heading_angle) > mc_params.wheels.hard_turn_threshold) {
      /* Hard Turn, heading angle very large */
      m_turning_state = turning_state::HARD_TURN;
   } else if (m_turning_state == turning_state::NO_TURN &&
           Abs(heading_angle) > mc_params.wheels.soft_turn_threshold) {
      /* Soft Turn, heading angle in between the two cases */
      m_turning_state = turning_state::SOFT_TURN;
   }

   /* Wheel speeds based on current turning state */
   argos::Real speed1, speed2;
   switch (m_turning_state) {
      case turning_state::NO_TURN: {
         /* Just go straight */
         speed1 = base_angular_wheel_speed;
         speed2 = base_angular_wheel_speed;
         break;
      }

      case turning_state::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
        argos::Real speed_factor = (mc_params.wheels.hard_turn_threshold - Abs(heading_angle)) / mc_params.wheels.hard_turn_threshold;
         speed1 = base_angular_wheel_speed - base_angular_wheel_speed * (1.0 - speed_factor);
         speed2 = base_angular_wheel_speed + base_angular_wheel_speed * (1.0 - speed_factor);
         break;
      }

      case turning_state::HARD_TURN: {
         /* Opposite wheel speeds */
         speed1 = -mc_params.wheels.max_speed;
         speed2 =  mc_params.wheels.max_speed;
         break;
      }
   }

   /* Apply the calculated speeds to the appropriate wheels */
   argos::Real left_wheel_speed, right_wheel_speed;
   if(heading_angle > argos::CRadians::ZERO) {
      /* Turn Left */
      left_wheel_speed  = speed1;
      right_wheel_speed = speed2;
   }
   else {
      /* Turn Right */
      left_wheel_speed  = speed2;
      right_wheel_speed = speed1;
   }
   /* Finally, set the wheel speeds */
   m_wheels->SetLinearVelocity(left_wheel_speed, right_wheel_speed);
} /* set_wheel_speeds() */

NS_END(fordyca);
