/**
 * @file birtd_dpo_controller.cpp
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
#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"

#include <fstream>

#include "rcppsw/ta/bi_tdgraph_executive.hpp"

#include "fordyca/config/depth2/controller_repository.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth2/task_executive_builder.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
birtd_dpo_controller::birtd_dpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth2.birtd_dpo") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void birtd_dpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && -1 == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id(),
            block()->robot_id());
  dpo_perception()->update(nullptr);
  executive()->run();
  saa()->steer_force2D_apply();
  ndc_pop();
} /* control_step() */

void birtd_dpo_controller::init(ticpp::Element& node) {
  base_controller::init(node);
  ndc_push();
  ER_INFO("Initializing");

  config::depth2::controller_repository config_repo;
  config_repo.parse_all(node);
  if (!config_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  shared_init(config_repo);
  private_init(config_repo);

  ER_INFO("Initialization finished");
  ndc_pop();
} /* init() */

void birtd_dpo_controller::private_init(
    const config::depth2::controller_repository& config_repo) {
  /*
   * Rebind executive to use depth2 task decomposition graph instead of depth1
   * version.
   */
  executive(task_executive_builder(block_sel_matrix(),
                                   cache_sel_matrix(),
                                   saa(),
                                   perception())(config_repo, rng()));

  /*
   * Set task alloction callback, rebind task abort callback (original was lost
   * when we replaced the executive).
   */
  executive()->task_start_notify(std::bind(&birtd_dpo_controller::task_start_cb,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
  executive()->task_abort_notify(std::bind(
      &birtd_dpo_controller::task_abort_cb, this, std::placeholders::_1));
} /* private_init() */

void birtd_dpo_controller::task_start_cb(const rta::polled_task* const task,
                                         const rta::ds::bi_tab* const) {
  /**
   * @brief Callback for task alloc. Needed to reset the task state of the
   * controller (not the task, which is handled by the executive) in the case
   * that the previous task was aborted. Not reseting this results in erroneous
   * handling of the newly allocated task as if it was aborted by the loop
   * functions, resulting in inconsistent state with the robot's executive. See
   * #532,#587.
   */
  if (tasks::task_status::ekAbortPending != task_status()) {
    task_status_update(tasks::task_status::ekRunning);
  }

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

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(birtd_dpo_controller,
                    "birtd_dpo_controller"); // NOLINT

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth2, controller, fordyca);
