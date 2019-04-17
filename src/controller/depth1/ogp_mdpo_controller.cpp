/**
 * @file ogp_mdpo_controller.cpp
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
#include "fordyca/controller/depth1/ogp_mdpo_controller.hpp"
#include "fordyca/controller/depth1/tasking_initializer.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/params/depth1/controller_repository.hpp"
#include "fordyca/params/oracle_params.hpp"
#include "fordyca/support/tasking_oracle.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"
#include "rcppsw/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ogp_mdpo_controller::Init(ticpp::Element& node) {
  base_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing");
  params::depth1::controller_repository param_repo;

  param_repo.parse_all(node);
  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  gp_mdpo_controller::shared_init(param_repo);
  oracle_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void ogp_mdpo_controller::oracle_init(void) {
  executive()->task_abort_notify(std::bind(
      &ogp_mdpo_controller::task_abort_cb, this, std::placeholders::_1));
  executive()->task_finish_notify(std::bind(
      &ogp_mdpo_controller::task_finish_cb, this, std::placeholders::_1));
} /* oracle_init() */

void ogp_mdpo_controller::task_abort_cb(rta::polled_task* task) {
  double oracle_est = boost::get<rta::time_estimate>(
                          mc_tasking_oracle->ask("exec_est." + task->name()))
                          .last_result();
  __rcsw_unused double old = task->task_exec_estimate().last_result();
  task->exec_estimate_update(oracle_est);
  ER_INFO("Update 'exec_est.%s' with oracular estimate %f on abort: %f -> %f",
          task->name().c_str(),
          oracle_est,
          old,
          task->task_exec_estimate().last_result());
} /* task_abort_cb() */

void ogp_mdpo_controller::task_finish_cb(rta::polled_task* task) {
  double oracle_est = boost::get<rta::time_estimate>(
                          mc_tasking_oracle->ask("exec_est." + task->name()))
                          .last_result();
  __rcsw_unused double old = task->task_exec_estimate().last_result();
  task->exec_estimate_update(oracle_est);
  ER_INFO("Update 'exec_est.%s' with oracular estimate %f on finish: %f -> %f",
          task->name().c_str(),
          oracle_est,
          old,
          task->task_exec_estimate().last_result());

  oracle_est = boost::get<rta::time_estimate>(
                   mc_tasking_oracle->ask("interface_est." + task->name()))
                   .last_result();
  old = task->task_interface_estimate(0).last_result();
  task->exec_estimate_update(oracle_est);
  ER_INFO(
      "Update 'interface_est.%s' with oracular estimate %f on finish: %f -> %f",
      task->name().c_str(),
      oracle_est,
      old,
      task->task_interface_estimate(0).last_result());
} /* task_finish_cb() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(ogp_mdpo_controller, "ogp_mdpo_controller");
#pragma clang diagnostic pop
NS_END(depth1, controller, fordyca);
