/**
v * \file generalist.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/tasks/d0/generalist.hpp"

#include "cosm/subsystem/sensing_subsystem.hpp"

#include "fordyca/controller/cognitive/d0/events/block_vanished.hpp"
#include "fordyca/controller/cognitive/d0/events/free_block_pickup.hpp"
#include "fordyca/controller/cognitive/d0/events/nest_block_drop.hpp"
#include "fordyca/controller/cognitive/d1/events/block_vanished.hpp"
#include "fordyca/controller/cognitive/d1/events/free_block_pickup.hpp"
#include "fordyca/controller/cognitive/d1/events/nest_block_drop.hpp"
#include "fordyca/controller/cognitive/d2/events/block_vanished.hpp"
#include "fordyca/controller/cognitive/d2/events/free_block_pickup.hpp"
#include "fordyca/controller/cognitive/d2/events/nest_block_drop.hpp"
#include "fordyca/fsm/d0/free_block_to_nest_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, tasks, d0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
generalist::generalist(const cta::config::task_alloc_config* const config,
                       std::unique_ptr<cta::taskable> mechanism)
    : ER_CLIENT_INIT("fordyca.tasks.d0.generalist"),
      foraging_task(kGeneralistName, config, std::move(mechanism)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double generalist::abort_prob_calc(void) {
  return executable_task::abort_prob();
} /* abort_prob_calc() */

rtypes::timestep generalist::current_time(void) const {
  return dynamic_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism())
      ->sensing()
      ->tick();
} /* current_time() */

/*******************************************************************************
 * Event Handling
 ******************************************************************************/
void generalist::accept(fccd0::events::nest_block_drop& visitor) {
  visitor.visit(*this);
}
void generalist::accept(fccd0::events::free_block_pickup& visitor) {
  visitor.visit(*this);
}
void generalist::accept(fccd0::events::block_vanished& visitor) {
  auto& fsm = *static_cast<fsm::d0::free_block_to_nest_fsm*>(mechanism());
  visitor.visit(fsm);
}

void generalist::accept(fccd1::events::nest_block_drop& visitor) {
  auto& fsm = *static_cast<fsm::d0::free_block_to_nest_fsm*>(mechanism());
  static_cast<fccd0::events::nest_block_drop&>(visitor).visit(fsm);
}
void generalist::accept(fccd1::events::free_block_pickup& visitor) {
  auto& fsm = *static_cast<fsm::d0::free_block_to_nest_fsm*>(mechanism());
  static_cast<fccd0::events::free_block_pickup&>(visitor).visit(fsm);
}
void generalist::accept(fccd1::events::block_vanished& visitor) {
  auto& fsm = *static_cast<fsm::d0::free_block_to_nest_fsm*>(mechanism());
  static_cast<fccd0::events::block_vanished&>(visitor).visit(fsm);
}

void generalist::accept(fccd2::events::nest_block_drop& visitor) {
  auto& fsm = *static_cast<fsm::d0::free_block_to_nest_fsm*>(mechanism());
  static_cast<fccd0::events::nest_block_drop&>(visitor).visit(fsm);
}
void generalist::accept(fccd2::events::free_block_pickup& visitor) {
  auto& fsm = *static_cast<fsm::d0::free_block_to_nest_fsm*>(mechanism());
  static_cast<fccd0::events::free_block_pickup&>(visitor).visit(fsm);
}
void generalist::accept(fccd2::events::block_vanished& visitor) {
  auto& fsm = *static_cast<fsm::d0::free_block_to_nest_fsm*>(mechanism());
  static_cast<fccd0::events::block_vanished&>(visitor).visit(fsm);
}

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    is_exploring_for_goal,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);
RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    is_vectoring_to_goal,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    goal_acquired,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    acquisition_goal,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    block_transport_goal,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    acquisition_loc3D,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    explore_loc3D,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    vector_loc3D,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    entity_acquired_id,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

/*******************************************************************************
 * Block Carrying
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(
    generalist,
    block_drop_strategy,
    *static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism()),
    const);

/*******************************************************************************
 * Block Transportation
 ******************************************************************************/
bool generalist::is_phototaxiing_to_goal(bool include_ca) const {
  return static_cast<fsm::d0::free_block_to_nest_fsm*>(polled_task::mechanism())
      ->is_phototaxiing_to_goal(include_ca);
} /* is_phototaxiing_to_goal() */

NS_END(d0, tasks, fordyca);
