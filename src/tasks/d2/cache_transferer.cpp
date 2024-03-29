/**
 * \file cache_transferer.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/tasks/d2/cache_transferer.hpp"

#include "fordyca/controller/cognitive/d2/events/block_found.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_block_drop.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_vanished.hpp"
#include "fordyca/controller/cognitive/d2/events/cached_block_pickup.hpp"
#include "fordyca/fsm/d2/cache_transferer_fsm.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_transferer::cache_transferer(
    const struct cta::config::task_alloc_config* config,
    std::unique_ptr<cta::taskable> mechanism)
    : foraging_task(kCacheTransfererName, config, std::move(mechanism)),
      ER_CLIENT_INIT("fordyca.tasks.d2.cache_transferer") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_transferer::task_start(cta::taskable_argument* const) {
  foraging_signal_argument a(fsm::foraging_signal::ekACQUIRE_CACHED_BLOCK);
  cta::polled_task::mechanism()->task_start(&a);
} /* task_start() */

double cache_transferer::abort_prob_calc(void) {
  if (-1 == active_interface()) {
    return cta::abort_probability::kMIN_ABORT_PROB;
  } else {
    return executable_task::abort_prob();
  }
} /* abort_prob_calc() */

rtypes::timestep
cache_transferer::interface_time_calc(size_t,
                                      const rtypes::timestep& start_time) {
  return current_time() - start_time;
} /* interface_time_calc() */

void cache_transferer::active_interface_update(int) {
  auto* fsm = static_cast<fsm::d2::cache_transferer_fsm*>(mechanism());

  if (fsm->is_acquiring_src_cache()) {
    if (fsm->goal_acquired() && interface_in_prog(0)) {
      interface_exit(0);
      interface_time_mark_finish(0);
      ER_TRACE("Interface0 finished at timestep %zuu", current_time().v());
    }
    if (!interface_in_prog(0)) {
      interface_enter(0);
      interface_time_mark_start(0);
      ER_TRACE("Interface0 start at timestep %zu", current_time().v());
    }
    ER_TRACE("Interface0 time: %zu", interface_time(0).v());
  } else if (fsm->is_acquiring_dest_cache()) {
    if (fsm->goal_acquired() && interface_in_prog(1)) {
      interface_exit(1);
      interface_time_mark_finish(1);
      ER_TRACE("Interface1 finished at timestep %zu", current_time().v());
    }
    if (!interface_in_prog(1)) {
      interface_enter(1);
      interface_time_mark_start(1);
      ER_TRACE("Interface1 start at timestep %zu", current_time().v());
    }
    ER_TRACE("Interface1 time: %zu", interface_time(1).v());
  }
} /* active_interface_update() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void cache_transferer::accept(fccd2::events::cache_block_drop& visitor) {
  auto& fsm = *static_cast<ffsm::block_to_goal_fsm*>(mechanism());
  static_cast<fccd1::events::cache_block_drop&>(visitor).visit(fsm);
}

void cache_transferer::accept(fccd2::events::cached_block_pickup& visitor) {
  auto& fsm = *static_cast<ffsm::block_to_goal_fsm*>(mechanism());
  static_cast<fccd1::events::cached_block_pickup&>(visitor).visit(fsm);
}

void cache_transferer::accept(fccd2::events::cache_vanished& visitor) {
  auto& fsm = *static_cast<ffsm::block_to_goal_fsm*>(mechanism());
  static_cast<fccd1::events::cache_vanished&>(visitor).visit(fsm);
}

/*******************************************************************************
 * Block Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    is_exploring_for_goal,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);
RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    is_vectoring_to_goal,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    goal_acquired,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    acquisition_goal,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    block_transport_goal,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    acquisition_loc3D,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    vector_loc3D,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    explore_loc3D,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    entity_acquired_id,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

/*******************************************************************************
 * Block Carrying
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(
    cache_transferer,
    block_drop_strategy,
    *static_cast<fsm::d2::cache_transferer_fsm*>(polled_task::mechanism()),
    const);

NS_END(d2, tasks, fordyca);
