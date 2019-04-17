/**
 * @file grp_dpo_controller.cpp
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
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth2/tasking_initializer.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/params/depth2/controller_repository.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

#include "rcppsw/ta/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
grp_dpo_controller::grp_dpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth2.grp_dpo") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void grp_dpo_controller::ControlStep(void) {
  ndc_pusht();
  if (nullptr != block()) {
    ER_ASSERT(-1 != block()->robot_id(),
              "Carried block%d has robot id=%d",
              block()->id(),
              block()->robot_id());
  }
  dpo_perception()->update();

  executive()->run();
  ndc_pop();
} /* ControlStep() */

void grp_dpo_controller::Init(ticpp::Element& node) {
  base_controller::Init(node);
  ndc_push();
  ER_INFO("Initializing");

  params::depth2::controller_repository param_repo;
  param_repo.parse_all(node);
  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  shared_init(param_repo);
  private_init(param_repo);

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void grp_dpo_controller::private_init(
    const params::depth2::controller_repository& param_repo) {
  /*
   * Rebind executive to use depth2 task decomposition graph instead of depth1
   * version.
   */
  executive(tasking_initializer(block_sel_matrix(),
                                cache_sel_matrix(),
                                saa_subsystem(),
                                perception())(param_repo));

  /*
   * Set task alloction callback, rebind task abort callback (original was lost
   * when we replaced the executive).
   */
  executive()->task_alloc_notify(std::bind(&grp_dpo_controller::task_alloc_cb,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
  executive()->task_abort_notify(std::bind(
      &grp_dpo_controller::task_abort_cb, this, std::placeholders::_1));
} /* private_init() */

void grp_dpo_controller::task_alloc_cb(const rta::polled_task* const task,
                                       const rta::bi_tab* const) {
  if (!m_bsel_exception_added) {
    block_sel_matrix()->sel_exceptions_clear();
  }
  m_bsel_exception_added = false;

  /*
   * We only care about the cache selection exceptions for the cache transferer
   * task. If that is not the task we just allocated ourself, then even if we
   * just finished the cache transferer task and added a cache to one of the
   * exception lists, remove it.
   */
  if (!m_csel_exception_added ||
      task->name() != tasks::depth2::foraging_task::kCacheTransfererName) {
    cache_sel_matrix()->sel_exceptions_clear();
  }
  m_bsel_exception_added = false;
  m_csel_exception_added = false;
} /* task_alloc_cb() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(grp_dpo_controller,
                    "grp_dpo_controller"); // NOLINT
#pragma clang diagnostic pop

NS_END(depth2, controller, fordyca);
