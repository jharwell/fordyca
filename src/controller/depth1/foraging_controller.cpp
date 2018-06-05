/**
 * @file foraging_controller.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/perception_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/params/depth0/stateful_foraging_repository.hpp"
#include "fordyca/params/depth1/task_allocation_params.hpp"
#include "fordyca/params/depth1/task_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/collector.hpp"
#include "fordyca/tasks/generalist.hpp"
#include "fordyca/tasks/harvester.hpp"

#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller::foraging_controller(void)
    : depth0::stateful_foraging_controller(),
      m_metric_store(),
      m_executive(),
      m_harvester(),
      m_collector(),
      m_generalist() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_controller::ControlStep(void) {
  perception()->update(depth0::stateful_foraging_controller::los());
  m_metric_store.reset();

  saa_subsystem()->actuation()->block_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->block_throttle_update();

  m_executive->run();
} /* ControlStep() */

void foraging_controller::Init(ticpp::Element& node) {
  params::depth1::task_repository task_repo;
  params::depth0::stateful_foraging_repository stateful_repo;

  depth0::stateful_foraging_controller::Init(node);
  ER_NOM("Initializing depth1 controller");

  task_repo.parse_all(node);
  server_handle()->log_stream() << task_repo;
  stateful_repo.parse_all(node);
  server_handle()->log_stream() << stateful_repo;

  ER_ASSERT(task_repo.validate_all(),
            "FATAL: Not all FSM parameters were validated");
  ER_ASSERT(stateful_repo.validate_all(),
            "FATAL: Not all task parameters were validated");

  auto* p = task_repo.parse_results<params::depth1::task_allocation_params>();

  /* Put in new depth1 sensors and perception, ala strategy pattern */
  saa_subsystem()->sensing(std::make_shared<depth1::sensing_subsystem>(
      stateful_repo.parse_results<struct params::sensing_params>(),
      &saa_subsystem()->sensing()->sensor_list()));

  perception(rcppsw::make_unique<perception_subsystem>(
      server(),
      stateful_repo.parse_results<params::perception_params>(),
      GetId()));

  /* initialize task decomposition graph */
  std::unique_ptr<task_allocation::taskable> collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          stateful_repo.parse_results<params::fsm_params>(),
          base_foraging_controller::server(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());
  m_collector =
      rcppsw::make_unique<tasks::collector>(&p->executive, collector_fsm);

  std::unique_ptr<task_allocation::taskable> harvester_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_existing_cache_fsm>(
          stateful_repo.parse_results<params::fsm_params>(),
          base_foraging_controller::server(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());
  m_harvester =
      rcppsw::make_unique<tasks::harvester>(&p->executive, harvester_fsm);

  std::unique_ptr<task_allocation::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_foraging_fsm>(
          stateful_repo.parse_results<params::fsm_params>(),
          base_foraging_controller::server(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());
  m_generalist =
      rcppsw::make_unique<tasks::generalist>(&p->executive, generalist_fsm);

  m_generalist->partition1(m_harvester.get());
  m_generalist->partition2(m_collector.get());
  m_generalist->parent(m_generalist.get());
  m_generalist->set_partitionable();

  m_harvester->parent(m_generalist.get());
  m_collector->parent(m_generalist.get());

  m_executive = rcppsw::make_unique<task_allocation::polled_executive>(
      base_foraging_controller::server(), m_generalist.get());

  m_executive->task_abort_cleanup(std::bind(
      &foraging_controller::task_abort_cleanup, this, std::placeholders::_1));

  m_executive->task_alloc_notify(std::bind(
      &foraging_controller::task_alloc_notify, this, std::placeholders::_1));

  m_executive->task_finish_notify(std::bind(
      &foraging_controller::task_finish_notify, this, std::placeholders::_1));

  if (p->exec_estimates.enabled) {
    m_generalist->init_random(p->exec_estimates.generalist_range.GetMin(),
                              p->exec_estimates.generalist_range.GetMax());
    m_harvester->init_random(p->exec_estimates.harvester_range.GetMin(),
                             p->exec_estimates.harvester_range.GetMax());
    m_collector->init_random(p->exec_estimates.collector_range.GetMin(),
                             p->exec_estimates.collector_range.GetMax());
  }
  ER_NOM("depth1 controller initialization finished");
} /* Init() */

__pure tasks::foraging_task* foraging_controller::current_task(void) const {
  return dynamic_cast<tasks::foraging_task*>(m_executive->current_task());
} /* current_task() */

bool foraging_controller::is_transporting_to_nest(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_transporting_to_nest();
  }
  return false;
} /* is_transporting_to_nest() */

/*******************************************************************************
 * Executive Callbacks
 ******************************************************************************/
void foraging_controller::task_abort_cleanup(
    task_allocation::executable_task* const) {
  m_metric_store.task_aborted = true;
} /* task_abort_cleanup() */

void foraging_controller::task_alloc_notify(
    task_allocation::executable_task* const task) {
  m_metric_store.task_alloc = true;
  if (nullptr == current_task() ||
      task->name() != m_executive->last_task()->name()) {
    m_metric_store.alloc_sw = true;
  }
} /* task_alloc_notify() */

void foraging_controller::task_finish_notify(
    task_allocation::executable_task* const task) {
  m_metric_store.last_task_exec_time = task->exec_time();
  m_metric_store.task_finish = true;
} /* task_finish_notify() */

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
bool foraging_controller::employed_partitioning(void) const {
  ER_ASSERT(nullptr != current_task(),
            "FATAL: Have not yet employed partitioning?");

  auto* task = dynamic_cast<task_allocation::executable_task*>(current_task());
  task_allocation::partitionable_task* p = nullptr;

  if (!task->is_partitionable()) {
    p = dynamic_cast<task_allocation::partitionable_task*>(task->parent());
  } else {
    p = dynamic_cast<task_allocation::partitionable_task*>(task);
  }
  ER_ASSERT(nullptr != p, "FATAL: Not an executable task?");
  return p->employed_partitioning();
} /* employed_partitioning() */

std::string foraging_controller::subtask_selection(void) const {
  ER_ASSERT(nullptr != current_task(),
            "FATAL: Have not yet selected a subtask?");
  /*
   * If we get into this function, then employed_partitioning() must have
   * returned TRUE, so we can just return the name of the current task, which
   * MUST be a subtask.
   */
  return current_task_name();
} /* subtask_selection() */

std::string foraging_controller::current_task_name(void) const {
  return dynamic_cast<task_allocation::logical_task*>(current_task())->name();
} /* current_task_name() */

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
using depth1_foraging_controller = foraging_controller;

REGISTER_CONTROLLER(depth1_foraging_controller,
                    "depth1_foraging_controller"); // NOLINT

NS_END(depth1, controller, fordyca);
