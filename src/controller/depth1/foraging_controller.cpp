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
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/perception_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/depth1/tasking_initializer.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/depth1/param_repository.hpp"
#include "fordyca/params/sensing_params.hpp"

#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/executive_params.hpp"
#include "rcppsw/task_allocation/partitionable_task.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"
#include "rcppsw/task_allocation/task_decomposition_graph.hpp"

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
      m_task_collator(),
      m_executive(),
      m_graph() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_controller::ControlStep(void) {
  perception()->update(depth0::stateful_foraging_controller::los());
  m_task_collator.reset();

  saa_subsystem()->actuation()->block_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->block_throttle_update();

  m_executive->run();
} /* ControlStep() */

void foraging_controller::Init(ticpp::Element& node) {
  params::depth1::param_repository param_repo(client::server_ref());

  /*
   * Note that we do not call \ref stateful_foraging_controller::Init()--there
   * is nothing in there that we need.
   */
  base_foraging_controller::Init(node);
  ER_NOM("Initializing depth1 foraging controller");

  param_repo.parse_all(node);
  server_ptr()->log_stream() << param_repo;

  ER_ASSERT(param_repo.validate_all(),
            "FATAL: Not all task parameters were validated");

  /* Put in new depth1 sensors and perception, ala strategy pattern */
  saa_subsystem()->sensing(std::make_shared<depth1::sensing_subsystem>(
      param_repo.parse_results<struct params::sensing_params>(),
      &saa_subsystem()->sensing()->sensor_list()));

  perception(rcppsw::make_unique<perception_subsystem>(
      client::server_ref(),
      param_repo.parse_results<params::perception_params>(),
      GetId()));

  /* initialize tasking */
  m_executive = tasking_initializer(client::server_ref(),
                                    saa_subsystem(),
                                    perception())(&param_repo);
  m_executive->task_abort_cleanup(std::bind(
            &foraging_controller::task_abort_cleanup, this, std::placeholders::_1));

  m_executive->task_alloc_notify(std::bind(
      &foraging_controller::task_alloc_notify, this, std::placeholders::_1));

  m_executive->task_finish_notify(std::bind(
      &foraging_controller::task_finish_notify, this, std::placeholders::_1));

  ER_NOM("Depth1 foraging controller initialization finished");
} /* Init() */

__rcsw_pure std::shared_ptr<tasks::base_foraging_task> foraging_controller::
    current_task(void) const {
  return std::dynamic_pointer_cast<tasks::base_foraging_task>(
      m_executive->current_task());
} /* current_task() */

/*******************************************************************************
 * Executive Callbacks
 ******************************************************************************/
void foraging_controller::task_abort_cleanup(const ta::task_graph_vertex&) {
  m_task_collator.task_aborted(true);
} /* task_abort_cleanup() */

void foraging_controller::task_alloc_notify(const ta::task_graph_vertex& task) {
  m_task_collator.has_new_allocation(true);
  if (nullptr == current_task() ||
      task->name() != m_executive->last_task()->name()) {
    m_task_collator.allocation_changed(true);
  }
} /* task_alloc_notify() */

void foraging_controller::task_finish_notify(const ta::task_graph_vertex& task) {
  m_task_collator.task_last_exec_time(task->exec_time());
  m_task_collator.task_finished(true);
} /* task_finish_notify() */

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
bool foraging_controller::employed_partitioning(void) const {
  ER_ASSERT(nullptr != current_task(), "FATAL: Have not yet executed a task?");

  auto task = std::dynamic_pointer_cast<ta::executable_task>(current_task());
  auto partitionable = std::dynamic_pointer_cast<ta::partitionable_task>(task);

  if (!task->is_partitionable()) {
    partitionable = std::dynamic_pointer_cast<ta::partitionable_task>(
        m_executive->parent_task(
            std::dynamic_pointer_cast<ta::executable_task>(current_task())));
  }
  ER_ASSERT(nullptr != partitionable, "FATAL: Not partitionable task?");
  return partitionable->employed_partitioning();
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
  return std::dynamic_pointer_cast<ta::logical_task>(current_task())->name();
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
