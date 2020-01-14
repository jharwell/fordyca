/**
 * \file bitd_dpo_controller.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"

#include <fstream>

#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/subsystem/config/sensing_subsystem2D_config.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"

#include "fordyca/config/block_sel/block_sel_matrix_config.hpp"
#include "fordyca/config/cache_sel/cache_sel_matrix_config.hpp"
#include "fordyca/config/depth1/controller_repository.hpp"
#include "fordyca/config/saa_xml_names.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth1/task_executive_builder.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/tasks/base_foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bitd_dpo_controller::bitd_dpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth1.bitd_dpo"),
      m_cache_sel_matrix(),
      m_executive() {}

bitd_dpo_controller::~bitd_dpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bitd_dpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() &&
              rtypes::constants::kNoUUID == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->robot_id().v());

  dpo_perception()->update(nullptr);
  executive()->run();
  saa()->steer_force2D_apply();
  ndc_pop();
} /* control_step() */

void bitd_dpo_controller::init(ticpp::Element& node) {
  base_controller::init(node);

  ndc_push();
  ER_INFO("Initializing...");
  config::depth1::controller_repository config_repo;

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

void bitd_dpo_controller::shared_init(
    const config::depth1::controller_repository& config_repo) {
  /* DPO perception subsystem, block selection matrix */
  dpo_controller::shared_init(config_repo);

  auto* cache_mat =
      config_repo.config_get<config::cache_sel::cache_sel_matrix_config>();
  auto* block_mat =
      config_repo.config_get<config::block_sel::block_sel_matrix_config>();

  /* cache selection matrix */
  m_cache_sel_matrix =
      std::make_unique<class cache_sel_matrix>(cache_mat, block_mat->nest);

  /*
   * Cache detection via ground sensors. This is *NOT* enabled by the
   * initialization in the base controller, and needs to happen here when we
   * have an XML repository with the correct configuration init.
   */
  auto saa_names = config::saa_xml_names();
  auto sensing_p =
      config_repo.config_get<csubsystem::config::sensing_subsystem2D_config>();
  auto ground = chal::sensors::ground_sensor(
      GetSensor<argos::CCI_FootBotMotorGroundSensor>(saa_names.ground_sensor),
      &sensing_p->ground);
  saa()->sensing()->replace(ground);
} /* shared_init() */

void bitd_dpo_controller::private_init(
    const config::depth1::controller_repository& config_repo) {
  /* task executive */
  m_executive = task_executive_builder(block_sel_matrix(),
                                       m_cache_sel_matrix.get(),
                                       saa(),
                                       perception())(config_repo, rng());
  executive()->task_abort_notify(std::bind(
      &bitd_dpo_controller::task_abort_cb, this, std::placeholders::_1));
  executive()->task_start_notify(std::bind(
      &bitd_dpo_controller::task_start_cb, this, std::placeholders::_1));
} /* private_init() */

void bitd_dpo_controller::task_abort_cb(const cta::polled_task*) {
  m_task_status = tasks::task_status::ekABORT_PENDING;
} /* task_abort_cb() */

void bitd_dpo_controller::task_start_cb(cta::polled_task* task) {
  if (tasks::task_status::ekABORT_PENDING != m_task_status) {
    m_task_status = tasks::task_status::ekRUNNING;
  }
  m_current_task = dynamic_cast<tasks::base_foraging_task*>(task);
} /* task_start_cb() */

const cta::ds::bi_tab* bitd_dpo_controller::active_tab(void) const {
  return m_executive->active_tab();
} /* active_tab() */

void bitd_dpo_controller::executive(
    std::unique_ptr<cta::bi_tdgraph_executive> executive) {
  m_executive = std::move(executive);
}

/*******************************************************************************
 * Block Transportation
 ******************************************************************************/
RCPPSW_WRAP_OVERRIDE_DEFP(bitd_dpo_controller,
                          block_transport_goal,
                          current_task(),
                          fsm::foraging_transport_goal::ekNONE,
                          const);

/*******************************************************************************
 * Goal Acquisition
 ******************************************************************************/
RCPPSW_WRAP_OVERRIDE_DEFP(
    bitd_dpo_controller,
    acquisition_goal,
    current_task(),
    cfmetrics::goal_acq_metrics::goal_type(fsm::foraging_acq_goal::ekNONE),
    const);

RCPPSW_WRAP_OVERRIDE_DEFP(bitd_dpo_controller,
                          goal_acquired,
                          current_task(),
                          false,
                          const);

/*******************************************************************************
 * Task Distribution Metrics
 ******************************************************************************/
int bitd_dpo_controller::current_task_depth(void) const {
  return executive()->graph()->vertex_depth(executive()->current_task());
} /* current_task_depth() */

int bitd_dpo_controller::current_task_id(void) const {
  if (nullptr != executive()->current_task()) {
    return executive()->graph()->vertex_id(executive()->current_task());
  }
  return -1;
} /* current_task_id() */

int bitd_dpo_controller::task_id(const std::string& task_name) const {
  auto v = executive()->graph()->find_vertex(task_name);
  return executive()->graph()->vertex_id(v);
} /* task_id() */

int bitd_dpo_controller::current_task_tab(void) const {
  return executive()->graph()->active_tab_id();
} /* current_task_tab() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(bitd_dpo_controller, "bitd_dpo_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth1, controller, fordyca);
