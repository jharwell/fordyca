/**
 * @file oracular_recpart_controller.cpp
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
#include "fordyca/controller/depth2/oracular_recpart_controller.hpp"
#include "fordyca/controller/depth2/tasking_initializer.hpp"
#include "fordyca/params/depth2/controller_repository.hpp"
#include "fordyca/support/tasking_oracle.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void oracular_recpart_controller::Init(ticpp::Element& node) {
  oracular_partitioning_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");
  params::depth2::controller_repository param_repo;

  param_repo.parse_all(node);

  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /*
   * Replace the executive/task decomposition graph via strategy pattern.
   */
  executive(tasking_initializer(true,
                                block_sel_matrix(),
                                cache_sel_matrix(),
                                saa_subsystem(),
                                perception())(&param_repo));
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(oracular_recpart_controller, "oracular_recpart_controller");
#pragma clang diagnostic pop
NS_END(depth2, controller, fordyca);
