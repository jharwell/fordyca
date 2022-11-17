/**
 * \file env_dynamics.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/tv/env_dynamics.hpp"

#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/argos/support/argos_swarm_manager.hpp"
#include "fordyca/argos/support/tv/config/env_dynamics_config.hpp"
#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
env_dynamics::env_dynamics(const fastv::config::env_dynamics_config* const config,
                           const fasupport::argos_swarm_manager* const lf,
                           carena::caching_arena_map* const map)
    : ER_CLIENT_INIT("fordyca.argos.support.tv.env_dynamics"),
      m_rda(&config->rda, lf),
      m_fb_pickup(map, &config->block_manip_penalty, "Free Block Pickup"),
      m_nest_drop(map, &config->block_manip_penalty, "Nest Block Pickup"),
      m_existing_cache(map, &config->cache_usage_penalty, "Existing Cache"),
      m_new_cache(map, &config->block_manip_penalty, "New Cache"),
      m_cache_site(map, &config->block_manip_penalty, "Cache Site") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::timestep env_dynamics::arena_block_manip_penalty(void) const {
  return penalty_handler(block_op_src::ekNEST_DROP)->penalty_calc(m_timestep);
} /* arena_block_manip_penalty() */

rtypes::timestep env_dynamics::cache_usage_penalty(void) const {
  return penalty_handler(cache_op_src::ekEXISTING_CACHE_PICKUP)
      ->penalty_calc(m_timestep);
} /* cache_usage_penalty() */

void env_dynamics::register_controller(const cpcontroller::controller2D& c) {
  m_rda.register_controller(c.entity_id());
} /* register_controller() */

void env_dynamics::unregister_controller(const cpcontroller::controller2D& c) {
  m_rda.unregister_controller(c.entity_id());
  penalties_flush(c);
} /* unregister_controller() */

bool env_dynamics::penalties_flush(const cpcontroller::controller2D& c) {
  bool aborted = false;
  for (auto& h : all_penalty_handlers()) {
    if (h->is_serving_penalty(c)) {
      ER_ASSERT(!aborted,
                "Controller serving penalties from more than one source");
      h->penalty_abort(c);
      aborted = true;
      ER_INFO("%s flushed from serving '%s' penalty",
              c.GetId().c_str(),
              h->name().c_str());
    }
  } /* for(&h..) */
  return aborted;
} /* penalties_flush() */

NS_END(tv, support, argos, fordyca);
