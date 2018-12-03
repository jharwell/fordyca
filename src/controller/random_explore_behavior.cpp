/**
 * @file random_explore_behavior.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of FORDYCA.
 *
 * FORDYCA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FORDYCA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FORDYCA.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/random_explore_behavior.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
random_explore_behavior::random_explore_behavior(
    controller::saa_subsystem* const saa)
    : explore_behavior(saa),
      ER_CLIENT_INIT("fordyca.controller.explore_behavior.random") {}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
__rcsw_pure bool random_explore_behavior::in_collision_avoidance(void) const {
  return m_in_avoidance;
} /* in_collision_avoidance() */

__rcsw_pure bool random_explore_behavior::entered_collision_avoidance(void) const {
  return m_entered_avoidance;
} /* entered_collision_avoidance() */

__rcsw_pure bool random_explore_behavior::exited_collision_avoidance(void) const {
  return m_exited_avoidance;
} /* exited_collision_avoidance() */

uint random_explore_behavior::collision_avoidance_duration(void) const {
  if (m_exited_avoidance) {
    return saa_subsystem()->sensing()->tick() - m_avoidance_start;
  }
  return 0;
} /* collision_avoidance_duration() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void random_explore_behavior::execute(void) {
  rmath::vector2d obs = saa_subsystem()->sensing()->find_closest_obstacle();
  saa_subsystem()->steering_force().avoidance(obs);
  saa_subsystem()->steering_force().wander();

  if (saa_subsystem()->sensing()->threatening_obstacle_exists()) {
    if (!m_in_avoidance) {
      if (!m_entered_avoidance) {
        m_entered_avoidance = true;
        m_avoidance_start = saa_subsystem()->sensing()->tick();
      }
    } else {
      m_entered_avoidance = false;
    }
    m_in_avoidance = true;

    ER_DEBUG("Found threatening obstacle: (%f, %f)@%f [%f]",
             obs.x(),
             obs.y(),
             obs.angle().value(),
             obs.length());
    saa_subsystem()->apply_steering_force(std::make_pair(false, false));
    saa_subsystem()->actuation()->leds_set_color(utils::color::kRED);
  } else {
    if (!m_exited_avoidance) {
      if (m_in_avoidance) {
        m_exited_avoidance = true;
      }
    } else {
      m_exited_avoidance = false;
    }
    m_in_avoidance = false;
    m_entered_avoidance = false; /* catches 1 timestep avoidances correctly */

    ER_DEBUG("No threatening obstacle found");
    saa_subsystem()->actuation()->leds_set_color(utils::color::kMAGENTA);
    rmath::vector2d force = saa_subsystem()->steering_force().value();
    /*
     * This can be 0 if the wander force is not active this timestep.
     */
    if (force.length() >= std::numeric_limits<double>::epsilon()) {
      saa_subsystem()->steering_force().value(
          saa_subsystem()->steering_force().value() * 0.7);
      saa_subsystem()->apply_steering_force(std::make_pair(false, false));
    }
  }
} /* execute() */

NS_END(controller, fordyca);
