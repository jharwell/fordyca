/**
 * \file ledtaxis.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/explore/ledtaxis.hpp"

#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

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
  saa()->apf2D().accum(saa()->apf2D().wander(rng()));

  if (auto obs = saa()->sensing()->proximity()->avg_prox_obj()) {
    inta_tracker()->state_enter();

    ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
             obs->to_str().c_str(),
             obs->angle().v(),
             obs->length());
    saa()->actuation()->diagnostics()->emit(
        chal::actuators::diagnostics::ekEXP_INTERFERENCE);
    saa()->apf2D().accum(saa()->apf2D().avoidance(*obs));

  } else {
    inta_tracker()->state_exit();

    ER_DEBUG("No threatening obstacle found");
    saa()->actuation()->diagnostics()->emit(
        chal::actuators::diagnostics::ekTAXIS);
    auto force = saa()->apf2D().phototaxis(
        saa()->sensing()->blobs()->readings(),
        carepr::light_type_index()[carepr::light_type_index::kCache]);
    saa()->apf2D().accum(force);
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
