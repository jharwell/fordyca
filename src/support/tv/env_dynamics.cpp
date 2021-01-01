/**
 * \file env_dynamics.cpp
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
#include "fordyca/support/tv/env_dynamics.hpp"

#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca//controller/foraging_controller.hpp"
#include "fordyca/config/tv/env_dynamics_config.hpp"
#include "fordyca/support/base_loop_functions.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
env_dynamics::env_dynamics(const config::tv::env_dynamics_config* const config,
                           const support::base_loop_functions* const lf,
                           carena::caching_arena_map* const map)
    : ER_CLIENT_INIT("fordyca.support.tv.env_dynamics"),
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

void env_dynamics::register_controller(const cpal::argos_controller2D_adaptor& c) {
  m_rda.register_controller(c.entity_id());
} /* register_controller() */

void env_dynamics::unregister_controller(
    const cpal::argos_controller2D_adaptor& c) {
  m_rda.unregister_controller(c.entity_id());
  penalties_flush(c);
} /* unregister_controller() */

bool env_dynamics::penalties_flush(const cpal::argos_controller2D_adaptor& c) {
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

NS_END(tv, support, fordyca);
