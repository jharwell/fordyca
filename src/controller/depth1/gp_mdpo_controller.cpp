/**
 * @file gp_mdpo_controller.cpp
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
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include <fstream>

#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth1/tasking_initializer.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/params/block_sel_matrix_params.hpp"
#include "fordyca/params/cache_sel_matrix_params.hpp"
#include "fordyca/params/depth1/controller_repository.hpp"
#include "fordyca/representation/base_block.hpp"

#include "rcppsw/task_allocation/bi_tdgraph.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
gp_mdpo_controller::gp_mdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth1.gp_mdpo"),
      m_cache_sel_matrix(),
      m_executive() {}

gp_mdpo_controller::~gp_mdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void gp_mdpo_controller::ControlStep(void) {
  ndc_pusht();
  if (nullptr != block()) {
    ER_ASSERT(-1 != block()->robot_id(),
              "Carried block%d has robot id=%d",
              block()->id(),
              block()->robot_id());
  }
  perception()->update();

  saa_subsystem()->actuation()->motion_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->motion_throttle_update(
      saa_subsystem()->sensing()->tick());

  m_task_aborted = false;
  executive()->run();
  ndc_pop();
} /* ControlStep() */

void gp_mdpo_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref mdpo_controller::Init()--there
   * is nothing in there that we need.
   */
  base_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");
  params::depth1::controller_repository param_repo;

  param_repo.parse_all(node);
  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  shared_init(param_repo);
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void gp_mdpo_controller::shared_init(
    const params::depth1::controller_repository& param_repo) {
  /* MDPO perception subsystem, block selection matrix */
  mdpo_controller::shared_init(param_repo);

  auto* cache_mat = param_repo.parse_results<params::cache_sel_matrix_params>();
  auto* block_mat = param_repo.parse_results<params::block_sel_matrix_params>();

  /* cache selection matrix */
  m_cache_sel_matrix =
      rcppsw::make_unique<class cache_sel_matrix>(cache_mat, block_mat->nest);

  /* task executive */
  m_executive = tasking_initializer(block_sel_matrix(),
                                    m_cache_sel_matrix.get(),
                                    saa_subsystem(),
                                    perception())(param_repo);
  executive()->task_abort_notify(std::bind(
      &gp_mdpo_controller::task_abort_cb, this, std::placeholders::_1));
} /* shared_init() */

void gp_mdpo_controller::task_abort_cb(const ta::polled_task*) {
  m_task_aborted = true;
} /* task_abort_cb() */

__rcsw_pure const ta::bi_tab* gp_mdpo_controller::active_tab(void) const {
  return m_executive->active_tab();
} /* active_tab() */

__rcsw_pure tasks::base_foraging_task* gp_mdpo_controller::current_task(void) {
  return dynamic_cast<tasks::base_foraging_task*>(
      m_executive.get()->current_task());
} /* current_task() */

__rcsw_pure const tasks::base_foraging_task* gp_mdpo_controller::current_task(
    void) const {
  return const_cast<gp_mdpo_controller*>(this)->current_task();
} /* current_task() */

void gp_mdpo_controller::executive(
    std::unique_ptr<ta::bi_tdgraph_executive> executive) {
  m_executive = std::move(executive);
}

/*******************************************************************************
 * Block Transportation
 ******************************************************************************/
TASK_WRAPPER_DEFINEC_PTR(transport_goal_type,
                         gp_mdpo_controller,
                         block_transport_goal,
                         current_task());

/*******************************************************************************
 * Goal Acquisition
 ******************************************************************************/
TASK_WRAPPER_DEFINEC_PTR(acquisition_goal_type,
                         gp_mdpo_controller,
                         acquisition_goal,
                         current_task());

TASK_WRAPPER_DEFINEC_PTR(bool, gp_mdpo_controller, goal_acquired, current_task());

/*******************************************************************************
 * Task Distribution Metrics
 ******************************************************************************/
int gp_mdpo_controller::current_task_depth(void) const {
  return executive()->graph()->vertex_depth(
      dynamic_cast<const ta::polled_task*>(current_task()));
} /* current_task_depth() */

int gp_mdpo_controller::current_task_id(void) const {
  return executive()->graph()->vertex_id(
      dynamic_cast<const ta::polled_task*>(current_task()));
} /* current_task_id() */

__rcsw_pure int gp_mdpo_controller::current_task_tab(void) const {
  return dynamic_cast<const ta::bi_tdgraph*>(executive()->graph())
      ->active_tab_id();
} /* current_task_tab() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(gp_mdpo_controller, "gp_mdpo_controller");
#pragma clang diagnostic pop
NS_END(depth1, controller, fordyca);
