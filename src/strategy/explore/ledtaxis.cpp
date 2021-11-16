/**
 * \file ledtaxis.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/strategy/explore/ledtaxis.hpp"

#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
ledtaxis::ledtaxis(const fstrategy::strategy_params* params, rmath::rng* rng)
    : foraging_strategy(params, rng),
      ER_CLIENT_INIT("fordyca.fsm.strategy.ledtaxis"),
      m_target(params->ledtaxis_target) {}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void ledtaxis::task_execute(void) {
  saa()->steer_force2D().accum(saa()->steer_force2D().wander(rng()));

  if (auto obs = saa()->sensing()->proximity()->avg_prox_obj()) {
    inta_tracker()->state_enter();

    ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
             obs->to_str().c_str(),
             obs->angle().v(),
             obs->length());
    saa()->actuation()->diagnostics()->emit(chal::actuators::diagnostics::ekEXP_INTERFERENCE);
    saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));

  } else {
    inta_tracker()->state_exit();

    ER_DEBUG("No threatening obstacle found");
    saa()->actuation()->diagnostics()->emit(chal::actuators::diagnostics::ekTAXIS);
    auto force = saa()->steer_force2D().phototaxis(
        saa()->sensing()->blobs()->readings(),
        carepr::light_type_index()[carepr::light_type_index::kCache]);
    saa()->steer_force2D().accum(force);
  }
} /* task_execute() */

bool ledtaxis::task_finished(void) const {
  rmath::vector2d accum;
  uint count = 0;

  if (!m_task_running) {
    return true;
  }

  for (auto& r : saa()->sensing()->blobs()->readings()) {
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

NS_END(explore, strategy, fordyca);
