/**
 * \file birtd_dpo_controller.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"

#include "fordyca/config/depth2/controller_repository.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth2/task_executive_builder.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/tasks/depth2/foraging_task.hpp"

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
  ER_ASSERT(!(nullptr != block() &&
              rtypes::constants::kNoUUID == block()->md()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());
  dpo_perception()->update(nullptr);

  /*
   * Execute the current task/allocate a new task/abort a task/etc and apply
   * steering forces if normal operation, otherwise handle abnormal operation
   * state.
   */
  supervisor()->run();

  ndc_pop();
} /* control_step() */

void birtd_dpo_controller::init(ticpp::Element& node) {
  foraging_controller::init(node);
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
  supervisor()->supervisee_update(executive());
} /* private_init() */

void birtd_dpo_controller::task_start_cb(cta::polled_task* const task,
                                         const cta::ds::bi_tab* const) {
  if (tasks::task_status::ekABORT_PENDING != task_status()) {
    task_status_update(tasks::task_status::ekRUNNING);
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

  /*
   * This is set in the BITD DPO controller's task start cb, but we don't use
   * that here, so it needs to also be here. See #622.
   */
  current_task(dynamic_cast<tasks::base_foraging_task*>(task));
} /* task_start_cb() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(birtd_dpo_controller,
                    "birtd_dpo_controller"); // NOLINT

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth2, controller, fordyca);
