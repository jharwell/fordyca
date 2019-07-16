/**
 * @file crw.cpp
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
#include "fordyca/fsm/expstrat/crw.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void crw::task_execute(void) {
  saa_subsystem()->steer2D_force_calc().wander();

  if (auto obs = saa_subsystem()->sensing()->avg_obstacle_within_prox()) {
    m_tracker.ca_enter();
    saa_subsystem()->steer2D_force_calc().avoidance(*obs);

    ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
             obs->to_str().c_str(),
             obs->angle().value(),
             obs->length());
    saa_subsystem()->actuation()->leds_set_color(rutils::color::kRED);
  } else {
    m_tracker.ca_exit();

    ER_DEBUG("No threatening obstacle found");
    saa_subsystem()->actuation()->leds_set_color(rutils::color::kMAGENTA);
    rmath::vector2d force = saa_subsystem()->steer2D_force_calc().value();
    /*
     * This can be 0 if the wander force is not active this timestep.
     */
    if (force.length() >= std::numeric_limits<double>::epsilon()) {
      saa_subsystem()->steer2D_force_calc().value(
          saa_subsystem()->steer2D_force_calc().value() * 0.7);
    }
  }
  saa_subsystem()->steer2D_force_apply();
} /* task_execute() */

NS_END(expstrat, fsm, fordyca);
