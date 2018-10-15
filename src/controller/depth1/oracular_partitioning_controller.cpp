/**
 * @file oracular_partitioning_controller.cpp
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
#include "fordyca/controller/depth1/oracular_partitioning_controller.hpp"
#include "fordyca/controller/depth1/tasking_initializer.hpp"
#include "fordyca/params/depth1/controller_repository.hpp"
#include "fordyca/support/tasking_oracle.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void oracular_partitioning_controller::Init(ticpp::Element& node) {
  greedy_partitioning_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");
  params::depth1::controller_repository param_repo;

  param_repo.parse_all(node);

  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  executive(tasking_initializer(true,
                                block_sel_matrix(),
                                cache_sel_matrix(),
                                saa_subsystem(),
                                perception())(&param_repo));
  executive()->task_abort_notify(
      std::bind(&oracular_partitioning_controller::task_abort_cb,
                this,
                std::placeholders::_1));
  executive()->task_finish_notify(
      std::bind(&oracular_partitioning_controller::task_finish_cb,
                this,
                std::placeholders::_1));
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void oracular_partitioning_controller::task_abort_cb(ta::polled_task* task) {
  double oracle_est = boost::get<ta::time_estimate>(
                          mc_tasking_oracle->ask("exec_est." + task->name()))
                          .last_result();
  double old = task->task_exec_estimate().last_result();
  task->exec_estimate_update(oracle_est);
  ER_INFO("Update 'exec_est.%s' with oracular estimate %f on abort: %f -> %f",
          task->name().c_str(),
          oracle_est,
          old,
          task->task_exec_estimate().last_result());
} /* task_abort_cb() */

void oracular_partitioning_controller::task_finish_cb(ta::polled_task* task) {
  double oracle_est = boost::get<ta::time_estimate>(
                          mc_tasking_oracle->ask("exec_est." + task->name()))
                          .last_result();
  double old = task->task_exec_estimate().last_result();
  task->exec_estimate_update(oracle_est);
  ER_INFO("Update 'exec_est.%s' with oracular estimate %f on finish: %f -> %f",
          task->name().c_str(),
          oracle_est,
          old,
          task->task_exec_estimate().last_result());
} /* task_finish_cb() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(oracular_partitioning_controller,
                    "oracular_partitioning_controller");
#pragma clang diagnostic pop
NS_END(depth1, controller, fordyca);
