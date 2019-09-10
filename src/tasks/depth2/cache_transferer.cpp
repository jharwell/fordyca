/**
 * @file cache_transferer.cpp
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
#include "fordyca/tasks/depth2/cache_transferer.hpp"

#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cache_vanished.hpp"
#include "fordyca/events/cached_block_pickup.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/fsm/depth2/cache_transferer_fsm.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_transferer::cache_transferer(
    const struct rta::config::task_alloc_config* config,
    std::unique_ptr<rta::taskable> mechanism)
    : foraging_task(kCacheTransfererName, config, std::move(mechanism)),
      ER_CLIENT_INIT("fordyca.tasks.depth2.cache_transferer") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

void cache_transferer::task_start(const rta::taskable_argument* const) {
  foraging_signal_argument a(fsm::foraging_signal::ekACQUIRE_CACHED_BLOCK);
  rta::polled_task::mechanism()->task_start(&a);
} /* task_start() */

double cache_transferer::abort_prob_calc(void) {
  if (-1 == active_interface()) {
    return rta::abort_probability::kMIN_ABORT_PROB;
  } else {
    return executable_task::abort_prob();
  }
} /* abort_prob_calc() */

rtypes::timestep cache_transferer::interface_time_calc(
    uint,
    const rtypes::timestep& start_time) {
  return rtypes::timestep(current_time() - start_time);
} /* interface_time_calc() */

void cache_transferer::active_interface_update(int) {
  auto* fsm = static_cast<fsm::depth2::cache_transferer_fsm*>(mechanism());

  if (fsm->is_acquiring_src_cache()) {
    if (fsm->goal_acquired() && interface_in_prog(0)) {
      interface_exit(0);
      interface_time_mark_finish(0);
      ER_TRACE("Interface0 finished at timestep %u", current_time().v());
    }
    if (!interface_in_prog(0)) {
      interface_enter(0);
      interface_time_mark_start(0);
      ER_TRACE("Interface0 start at timestep %u", current_time().v());
    }
    ER_TRACE("Interface0 time: %u", interface_time(0).v());
  } else if (fsm->is_acquiring_dest_cache()) {
    if (fsm->goal_acquired() && interface_in_prog(1)) {
      interface_exit(1);
      interface_time_mark_finish(1);
      ER_TRACE("Interface1 finished at timestep %u", current_time().v());
    }
    if (!interface_in_prog(1)) {
      interface_enter(1);
      interface_time_mark_start(1);
      ER_TRACE("Interface1 start at timestep %u", current_time().v());
    }
    ER_TRACE("Interface1 time: %u", interface_time(1).v());
  }
} /* active_interface_update() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void cache_transferer::accept(events::detail::cache_block_drop& visitor) {
  visitor.visit(*this);
}

void cache_transferer::accept(events::detail::cached_block_pickup& visitor) {
  visitor.visit(*this);
}

void cache_transferer::accept(events::detail::cache_vanished& visitor) {
  visitor.visit(*this);
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    is_exploring_for_goal,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);
RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    is_vectoring_to_goal,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    goal_acquired,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    acquisition_goal,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    block_transport_goal,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    acquisition_loc,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    current_vector_loc,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_OVERRIDE_DEF(
    cache_transferer,
    current_explore_loc,
    *static_cast<fsm::depth2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

NS_END(depth2, tasks, fordyca);
