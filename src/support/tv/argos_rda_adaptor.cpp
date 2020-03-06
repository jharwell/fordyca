/**
 * \file argos_rda_adaptor.cpp
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
/*
 * This is needed because without it boost instantiates static assertions that
 * verify that every possible handler<controller> instantiation is valid, which
 * includes checking for depth1 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT
#include "fordyca/support/tv/argos_rda_adaptor.hpp"

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "fordyca/support/swarm_iterator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
argos_rda_adaptor::argos_rda_adaptor(
    const ctv::config::robot_dynamics_applicator_config* config,
    const cpal::argos_sm_adaptor* const sm)
    : ER_CLIENT_INIT("fordyca.support.tv.argos_rda_adaptor"),
      robot_dynamics_applicator(config),
      mc_sm(sm) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double argos_rda_adaptor::avg_motion_throttle(void) const {
  double accum = 0.0;
  auto cb = [&](auto& controller) {
    accum += controller->applied_movement_throttle();
  };

  support::swarm_iterator::controllers<argos::CFootBotEntity,
                                       swarm_iterator::static_order>(
      mc_sm, cb, "foot-bot");
  return accum / mc_sm->GetSpace().GetEntitiesByType("foot-bot").size();
} /* avg_motion_throttle() */

void argos_rda_adaptor::update(void) {
  if (!motion_throttling_enabled()) {
    return;
  }
  rtypes::timestep t(mc_sm->GetSpace().GetSimulationClock());
  auto cb = [&](auto& controller) {
    motion_throttler(controller->entity_id())
        ->toggle(controller->is_carrying_block());
    motion_throttler(controller->entity_id())->update(t);
  };

  support::swarm_iterator::controllers<argos::CFootBotEntity,
                                       swarm_iterator::static_order>(
      mc_sm, cb, "foot-bot");
} /* update() */

NS_END(tv, support, fordyca);
