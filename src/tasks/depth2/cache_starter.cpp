/**
 * @file cache_starter.cpp
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
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/events/block_proximity.hpp"
#include "fordyca/events/block_vanished.hpp"
#include "fordyca/events/cache_proximity.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/events/free_block_pickup.hpp"

#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);
using acq_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_starter::cache_starter(const struct rta::config::task_alloc_config* config,
                             std::unique_ptr<rta::taskable> mechanism)
    : foraging_task(kCacheStarterName, config, std::move(mechanism)),
      ER_CLIENT_INIT("fordyca.tasks.depth2.cache_starter") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_starter::task_start(const rta::taskable_argument* const) {
  foraging_signal_argument a(controller::foraging_signal::ekACQUIRE_FREE_BLOCK);
  rta::polled_task::mechanism()->task_start(&a);
} /* task_start() */

__rcsw_pure double cache_starter::abort_prob_calc(void) {
  if (-1 == active_interface()) {
    return rta::abort_probability::kMIN_ABORT_PROB;
  } else {
    return executable_task::abort_prob();
  }
} /* abort_prob_calc() */

__rcsw_pure double cache_starter::interface_time_calc(uint interface,
                                                      double start_time) {
  ER_ASSERT(0 == interface, "Bad interface ID: %u", interface);
  return current_time() - start_time;
} /* interface_time_calc() */

void cache_starter::active_interface_update(int) {
  auto* fsm = static_cast<fsm::depth2::block_to_cache_site_fsm*>(mechanism());

  if (fsm->goal_acquired() &&
      transport_goal_type::ekCACHE_SITE == fsm->block_transport_goal()) {
    if (interface_in_prog(0)) {
      interface_exit(0);
      interface_time_mark_finish(0);
      ER_TRACE("Interface finished at timestep %f", current_time());
    }
    ER_TRACE("Interface time: %f", interface_time(0));
  } else if (transport_goal_type::ekCACHE_SITE == fsm->block_transport_goal()) {
    if (!interface_in_prog(0)) {
      interface_enter(0);
      interface_time_mark_start(0);
      ER_TRACE("Interface start at timestep %f", current_time());
    }
  }
} /* active_interface_update() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
TASK_WRAPPER_DEFINE_PTR(cache_starter::exp_status,
                        cache_starter,
                        is_exploring_for_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);
TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_starter,
                        is_vectoring_to_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);

TASK_WRAPPER_DEFINE_PTR(bool,
                        cache_starter,
                        goal_acquired,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);

TASK_WRAPPER_DEFINE_PTR(acq_goal_type,
                        cache_starter,
                        acquisition_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);

TASK_WRAPPER_DEFINE_PTR(transport_goal_type,
                        cache_starter,
                        block_transport_goal,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);

TASK_WRAPPER_DEFINE_PTR(rmath::vector2u,
                        cache_starter,
                        acquisition_loc,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);

TASK_WRAPPER_DEFINE_PTR(rmath::vector2u,
                        cache_starter,
                        current_vector_loc,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);

TASK_WRAPPER_DEFINE_PTR(rmath::vector2u,
                        cache_starter,
                        current_explore_loc,
                        static_cast<fsm::depth2::block_to_cache_site_fsm*>(
                            polled_task::mechanism()),
                        const);

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void cache_starter::accept(events::detail::free_block_drop& visitor) {
  visitor.visit(*this);
}
void cache_starter::accept(events::detail::free_block_pickup& visitor) {
  visitor.visit(*this);
}
void cache_starter::accept(events::detail::block_vanished& visitor) {
  visitor.visit(*this);
}
void cache_starter::accept(events::detail::block_proximity& visitor) {
  visitor.visit(*this);
}
void cache_starter::accept(events::detail::cache_proximity& visitor) {
  visitor.visit(*this);
}

NS_END(depth2, tasks, fordyca);
