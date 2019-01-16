/**
 * @file ogrp_mdpo_controller.cpp
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
#include "fordyca/controller/depth2/ogrp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/tasking_initializer.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/params/depth2/controller_repository.hpp"
#include "fordyca/params/oracle_params.hpp"
#include "fordyca/support/tasking_oracle.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/polled_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ogrp_mdpo_controller::Init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");
  params::depth2::controller_repository param_repo;

  param_repo.parse_all(node);
  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  shared_init(param_repo);

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void ogrp_mdpo_controller::shared_init(
    const params::depth2::controller_repository& param_repo) {
  /* create initial executive and bind task abort callback */
  gp_mdpo_controller::shared_init(param_repo);

  /*
   * Rebind executive to use depth2 decomposition graph instead of depth1
   * version.
   */
  executive(tasking_initializer(block_sel_matrix(),
                                cache_sel_matrix(),
                                saa_subsystem(),
                                perception())(param_repo));
  ogp_mdpo_controller::oracle_init();
} /* shared_init() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(ogrp_mdpo_controller, "ogrp_mdpo_controller");
#pragma clang diagnostic pop
NS_END(depth2, controller, fordyca);