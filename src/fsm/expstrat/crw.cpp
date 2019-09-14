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

#include "cosm/robots/footbot/footbot_actuation_subsystem.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw::crw(const fsm::expstrat::foraging_expstrat::params* const c_params,
         rmath::rng* rng)
    : crw(static_cast<crfootbot::footbot_saa_subsystem*>(c_params->saa), rng) {}

crw::crw(crfootbot::footbot_saa_subsystem* saa,
         rmath::rng* rng)
    : foraging_expstrat(saa, rng),
      ER_CLIENT_INIT("fordyca.fsm.expstrat.crw"),
      m_tracker(saa->sensing()) {}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void crw::task_execute(void) {
  saa()->steer_force2D().accum(saa()->steer_force2D().wander(rng()));

  if (auto obs = saa()->sensing()->proximity()->avg_prox_obj()) {
    m_tracker.ca_enter();
    saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));

    ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
             obs->to_str().c_str(),
             obs->angle().value(),
             obs->length());
    saa()->actuation()->leds()->set_color(-1, rutils::color::kRED);
  } else {
    m_tracker.ca_exit();

    ER_DEBUG("No threatening obstacle found");
    saa()->actuation()->leds()->set_color(-1, rutils::color::kMAGENTA);
    rmath::vector2d force = saa()->steer_force2D().value();
    /*
     * This can be 0 if the wander force is not active this timestep.
     */
    if (force.length() >= std::numeric_limits<double>::epsilon()) {
      saa()->steer_force2D().value(saa()->steer_force2D().value() * 0.7);
    }
  }
} /* task_execute() */

NS_END(expstrat, fsm, fordyca);
