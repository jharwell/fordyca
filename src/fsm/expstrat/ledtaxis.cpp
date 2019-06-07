/**
 * @file ledtaxis.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/fsm/expstrat/ledtaxis.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/support/light_type_index.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void ledtaxis::task_execute(void) {
  rmath::vector2d obs = saa_subsystem()->sensing()->find_closest_obstacle();
  saa_subsystem()->steer2D_force_calc().avoidance(obs);
  saa_subsystem()->steer2D_force_calc().wander();
  if (saa_subsystem()->sensing()->threatening_obstacle_exists()) {
    m_tracker.ca_enter();

    ER_DEBUG("Found threatening obstacle: (%f, %f)@%f [%f]",
             obs.x(),
             obs.y(),
             obs.angle().value(),
             obs.length());
    saa_subsystem()->actuation()->leds_set_color(rutils::color::kRED);
  } else {
    m_tracker.ca_exit();

    ER_DEBUG("No threatening obstacle found");
    saa_subsystem()->actuation()->leds_set_color(rutils::color::kMAGENTA);
    saa_subsystem()->steer2D_force_calc().phototaxis(
        saa_subsystem()->sensing()->blobs().readings(),
        support::light_type_index()[support::light_type_index::kCache]);
  }
  saa_subsystem()->steer2D_force_apply(std::make_pair(false, false));
} /* task_execute() */

bool ledtaxis::task_finished(void) const {
  rmath::vector2d accum;
  uint count = 0;

  if (!m_task_running) {
    return true;
  }

  for (auto &r : saa_subsystem()->sensing()->blobs().readings()) {
    if (r.color == m_target) {
      accum += r.vec;
      ++count;
    }
  } /* for(&r..) */

  /*
   * We are finished if we:
   *
   * - Are under the target light source
   * - There are no known light sources of the target color.
   */
  if (0 == count || accum.length() / count <= kARRIVAL_TOL) {
    m_task_running = false;
    return true;
  }
  return false;
} /* task_finished() */

NS_END(expstrat, fsm, fordyca);
