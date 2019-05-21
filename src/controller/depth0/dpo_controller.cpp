/**
 * @file dpo_controller.cpp
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
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include <fstream>

#include "fordyca/config/block_sel/block_sel_matrix_config.hpp"
#include "fordyca/config/depth0/dpo_controller_repository.hpp"
#include "fordyca/config/perception/perception_config.hpp"
#include "fordyca/config/sensing_config.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/expstrat/factory.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_controller::dpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth0.dpo"),
      m_block_sel_matrix(),
      m_perception(),
      m_fsm() {}

dpo_controller::~dpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_controller::fsm(std::unique_ptr<fsm::depth0::dpo_fsm> fsm) {
  m_fsm = std::move(fsm);
} /* fsm() */

void dpo_controller::perception(
    std::unique_ptr<base_perception_subsystem> perception) {
  m_perception = std::move(perception);
}

__rcsw_pure const repr::line_of_sight* dpo_controller::los(void) const {
  return static_cast<const dpo_perception_subsystem*>(m_perception.get())->los();
}
void dpo_controller::los(std::unique_ptr<repr::line_of_sight> new_los) {
  m_perception->los(std::move(new_los));
}

double dpo_controller::los_dim(void) const {
  return saa_subsystem()->sensing()->los_dim();
} /* los_dim() */

void dpo_controller::ControlStep(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && -1 == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id(),
            block()->robot_id());

  m_perception->update(nullptr);
  m_fsm->run();
  ndc_pop();
} /* ControlStep() */

void dpo_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref crw_controller::Init()--there
   * is nothing in there that we need.
   */
  base_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  config::depth0::dpo_controller_repository param_repo;
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

void dpo_controller::shared_init(
    const config::depth0::dpo_controller_repository& param_repo) {
  auto perception =
      param_repo.config_get<config::perception::perception_config>();
  auto block_matrix =
      param_repo.config_get<config::block_sel::block_sel_matrix_config>();

  /* DPO perception subsystem */
  m_perception = rcppsw::make_unique<dpo_perception_subsystem>(perception);

  /* block selection matrix */
  m_block_sel_matrix = rcppsw::make_unique<class block_sel_matrix>(block_matrix);
} /* shared_init() */

void dpo_controller::private_init(
    const config::depth0::dpo_controller_repository& param_repo) {
  auto* exp_config = param_repo.config_get<config::exploration_config>();
  fsm::expstrat::factory f;
  fsm::expstrat::base_expstrat::params p(saa_subsystem(),
                                         perception()->dpo_store());
  m_fsm = rcppsw::make_unique<fsm::depth0::dpo_fsm>(
      m_block_sel_matrix.get(),
      base_controller::saa_subsystem(),
      m_perception->dpo_store(),
      f.create(exp_config->strategy + "_block", &p));
} /* private_init() */

__rcsw_pure dpo_perception_subsystem* dpo_controller::dpo_perception(void) {
  return static_cast<dpo_perception_subsystem*>(m_perception.get());
} /* dpo_perception() */

__rcsw_pure const dpo_perception_subsystem* dpo_controller::dpo_perception(
    void) const {
  return static_cast<const dpo_perception_subsystem*>(m_perception.get());
} /* dpo_perception() */

void dpo_controller::Reset(void) {
  crw_controller::Reset();
  m_perception->reset();
} /* Reset() */

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_OVERRIDE_DEF(transport_goal_type,
                 dpo_controller,
                 block_transport_goal,
                 *m_fsm,
                 const);

FSM_OVERRIDE_DEF(acq_goal_type,
                 dpo_controller,
                 acquisition_goal,
                 *m_fsm,
                 const);

FSM_OVERRIDE_DEF(rmath::vector2u, dpo_controller, acquisition_loc, *m_fsm, const);
FSM_OVERRIDE_DEF(bool, dpo_controller, goal_acquired, *m_fsm, const);
FSM_OVERRIDE_DEF(dpo_controller::exp_status, dpo_controller, is_exploring_for_goal, *m_fsm, const);
FSM_OVERRIDE_DEF(bool, dpo_controller, is_vectoring_to_goal, *m_fsm, const);
FSM_OVERRIDE_DEF(rmath::vector2u,
                 dpo_controller,
                 current_vector_loc,
                 *m_fsm,
                 const);
FSM_OVERRIDE_DEF(rmath::vector2u,
                 dpo_controller,
                 current_explore_loc,
                 *m_fsm,
                 const);

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(dpo_controller, "dpo_controller");
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
