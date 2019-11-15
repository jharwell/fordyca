/**
 * @file irv_lf_adaptor.cpp
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
/*
 * This is needed because without it boost instantiates static assertions that
 * verify that every possible handler<controller> instantiation is valid, which
 * includes checking for depth1 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT
#include "fordyca/support/tv/irv_lf_adaptor.hpp"


#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "fordyca/support/base_loop_functions.hpp"
#include "fordyca/support/swarm_iterator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
irv_lf_adaptor::irv_lf_adaptor(const ctv::config::swarm_irv_manager_config* config,
                       const support::base_loop_functions* const lf)
    : ER_CLIENT_INIT("fordyca.support.tv.irv_lf_adaptor"),
      swarm_irv_manager(config),
      mc_lf(lf) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double irv_lf_adaptor::avg_motion_throttle(void) const {
  double accum = 0.0;
  auto cb = [&](auto& controller) {
    accum += controller->applied_movement_throttle();
  };

  support::swarm_iterator::controllers<argos::CFootBotEntity,
                                       swarm_iterator::static_order>(
                                           mc_lf, cb, "foot-bot");
  return accum / mc_lf->GetSpace().GetEntitiesByType("foot-bot").size();
} /* avg_motion_throttle() */

void irv_lf_adaptor::update(void) {
  if (!motion_throttling_enabled()) {
    return;
  }
  rtypes::timestep t(mc_lf->GetSpace().GetSimulationClock());
  auto cb = [&](auto& controller) {
    m_motion_throttling.at(controller->entity_id())
    .toggle(controller->is_carrying_block());
    m_motion_throttling.at(controller->entity_id()).update(t);
  };

  support::swarm_iterator::controllers<argos::CFootBotEntity,
                                       swarm_iterator::static_order>(
                                           mc_lf, cb, "foot-bot");
} /* update() */

NS_END(tv, support, fordyca);
