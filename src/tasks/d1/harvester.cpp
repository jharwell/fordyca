/**
 * \file harvester.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/d1/harvester.hpp"

#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/ta/config/task_alloc_config.hpp"

#include "fordyca/controller/cognitive/d1/events/block_vanished.hpp"
#include "fordyca/controller/cognitive/d1/events/cache_block_drop.hpp"
#include "fordyca/controller/cognitive/d1/events/cache_vanished.hpp"
#include "fordyca/controller/cognitive/d1/events/free_block_pickup.hpp"
#include "fordyca/controller/cognitive/d2/events/block_vanished.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_block_drop.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_vanished.hpp"
#include "fordyca/controller/cognitive/d2/events/free_block_pickup.hpp"
#include "fordyca/fsm/d1/block_to_existing_cache_fsm.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
harvester::harvester(const struct cta::config::task_alloc_config* config,
                     std::unique_ptr<cta::taskable> mechanism)
    : foraging_task(kHarvesterName, config, std::move(mechanism)),
      ER_CLIENT_INIT("fordyca.tasks.d1.harvester") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void harvester::task_start(cta::taskable_argument* const) {
  foraging_signal_argument a(fsm::foraging_signal::ekACQUIRE_FREE_BLOCK);
  cta::polled_task::mechanism()->task_start(&a);
} /* task_start() */

double harvester::abort_prob_calc(void) {
  /*
   * Harvesters always have a small chance of aborting their task when not at a
   * task interface. Having the harvester task un-abortable until AFTER it
   * acquires a block can cause it to get stuck and not switch to another task
   * if it cannot find a block anywhere. See FORDYCA#232.
   */
  if (-1 == active_interface()) {
    return cta::abort_probability::kMIN_ABORT_PROB;
  } else {
    return executable_task::abort_prob();
  }
} /* calc_abort_prob() */

rtypes::timestep
harvester::interface_time_calc(size_t interface,
                               const rtypes::timestep& start_time) {
  ER_ASSERT(0 == interface, "Bad interface ID: %zu", interface);
  return current_time() - start_time;
} /* interface_time_calc() */

void harvester::active_interface_update(int) {
  auto* fsm = static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());

  if (fsm->goal_acquired() && fsm::foraging_transport_goal::ekEXISTING_CACHE ==
                                  fsm->block_transport_goal()) {
    if (interface_in_prog(0)) {
      interface_exit(0);
      interface_time_mark_finish(0);
      ER_DEBUG("Interface finished at timestep %zu", current_time().v());
    }
    ER_TRACE("Interface time: %zu", interface_time(0).v());
  } else if (fsm::foraging_transport_goal::ekEXISTING_CACHE ==
             fsm->block_transport_goal()) {
    if (!interface_in_prog(0)) {
      interface_enter(0);
      interface_time_mark_start(0);
      ER_DEBUG("Interface start at timestep %zu", current_time().v());
    }
  }
} /* active_interface_update() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void harvester::accept(fccd1::events::cache_block_drop& visitor) {
  auto& fsm = *static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  visitor.visit(fsm);
}
void harvester::accept(fccd2::events::cache_block_drop& visitor) {
  auto& fsm = *static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  static_cast<fccd1::events::cache_block_drop&>(visitor).visit(fsm);
}

void harvester::accept(fccd1::events::free_block_pickup& visitor) {
  auto& fsm = *static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  visitor.visit(fsm);
}
void harvester::accept(fccd2::events::free_block_pickup& visitor) {
  auto& fsm = *static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  static_cast<fccd1::events::free_block_pickup&>(visitor).visit(fsm);
}

void harvester::accept(fccd1::events::cache_vanished& visitor) {
  auto& fsm = *static_cast<ffsm::block_to_goal_fsm*>(mechanism());
  visitor.visit(fsm);
}
void harvester::accept(fccd2::events::cache_vanished& visitor) {
  auto& fsm = *static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  static_cast<fccd1::events::cache_vanished&>(visitor).visit(fsm);
}

void harvester::accept(fccd1::events::block_vanished& visitor) {
  auto& fsm = *static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  visitor.visit(fsm);
}
void harvester::accept(fccd2::events::block_vanished& visitor) {
  auto& fsm = *static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  static_cast<fccd1::events::block_vanished&>(visitor).visit(fsm);
}

/*******************************************************************************
 * Block Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    is_exploring_for_goal,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);
RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    is_vectoring_to_goal,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    goal_acquired,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    acquisition_goal,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    block_transport_goal,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    acquisition_loc3D,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    explore_loc3D,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    vector_loc3D,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    entity_acquired_id,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

/*******************************************************************************
 * Block Carrying
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(
    harvester,
    block_drop_strategy,
    *static_cast<fsm::d1::block_to_existing_cache_fsm*>(polled_task::mechanism()),
    const);

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
bool harvester::task_at_interface(void) const {
  auto* fsm = static_cast<fsm::d1::block_to_existing_cache_fsm*>(mechanism());
  return fsm::foraging_transport_goal::ekEXISTING_CACHE ==
         fsm->block_transport_goal();
} /* task_at_interface()() */

NS_END(d1, tasks, fordyca);
