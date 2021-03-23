/**
 * \file dpo_controller.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"

#include <fstream>

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/config/nest_config.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"
#include "cosm/subsystem/perception/config/perception_config.hpp"
#include "cosm/spatial/strategy/nest_acq/factory.hpp"

#include "fordyca/config/block_sel/block_sel_matrix_config.hpp"
#include "fordyca/config/d0/dpo_controller_repository.hpp"
#include "fordyca/config/strategy/strategy_config.hpp"
#include "fordyca/controller/cognitive/block_sel_matrix.hpp"
#include "fordyca/controller/cognitive/dpo_perception_subsystem.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/strategy/explore/block_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_controller::dpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.d0.dpo"),
      m_block_sel_matrix(),
      m_perception(),
      m_fsm() {}

dpo_controller::~dpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_controller::fsm(std::unique_ptr<fsm::d0::dpo_fsm> fsm) {
  m_fsm = std::move(fsm);
} /* fsm() */

void dpo_controller::perception(
    std::unique_ptr<foraging_perception_subsystem> perception) {
  m_perception = std::move(perception);
}

double dpo_controller::los_dim(void) const {
  return perception()->los_dim();
} /* los_dim() */

void dpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && !block()->is_carried_by_robot()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());

  m_perception->update(nullptr);

  /*
   * Run the FSM and apply steering forces if normal operation, otherwise handle
   * abnormal operation state.
   */
  supervisor()->run();

  ndc_pop();
} /* control_step() */

void dpo_controller::init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref crw_controller::init()--there
   * is nothing in there that we need.
   */
  foraging_controller::init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  config::d0::dpo_controller_repository config_repo;
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

void dpo_controller::shared_init(
    const config::d0::dpo_controller_repository& config_repo) {
  auto* perception = config_repo.config_get<cspconfig::perception_config>();
  auto* block_matrix =
      config_repo.config_get<config::block_sel::block_sel_matrix_config>();
  auto* nest = config_repo.config_get<crepr::config::nest_config>();

  /* DPO perception subsystem */
  m_perception = std::make_unique<dpo_perception_subsystem>(perception);

  /* block selection matrix */
  m_block_sel_matrix =
      std::make_unique<class block_sel_matrix>(block_matrix, nest->center);
} /* shared_init() */

void dpo_controller::private_init(
    const config::d0::dpo_controller_repository& config_repo) {
  auto* strat_config = config_repo.config_get<fcstrategy::strategy_config>();

  fstrategy::foraging_strategy::params strategy_params(
      saa(), nullptr, nullptr, nullptr, rutils::color());
  fsm::fsm_ro_params fsm_ro_params = {
    .bsel_matrix = block_sel_matrix(),
    .csel_matrix = nullptr,
    .store = perception()->dpo_store(),
    .strategy_config = *strat_config
  };
  m_fsm = std::make_unique<fsm::d0::dpo_fsm>(
      &fsm_ro_params,
      saa(),
      fsexplore::block_factory().create(strat_config->explore.block_strategy,
                                        &strategy_params,
                                        rng()),
      csstrategy::nest_acq::factory().create(strat_config->nest_acq.strategy,
                                             saa(),
                                             rng()),
      rng());

  /* Set DPO FSM supervision */
  supervisor()->supervisee_update(m_fsm.get());
} /* private_init() */

dpo_perception_subsystem* dpo_controller::dpo_perception(void) {
  return static_cast<dpo_perception_subsystem*>(m_perception.get());
} /* dpo_perception() */

const dpo_perception_subsystem* dpo_controller::dpo_perception(void) const {
  return static_cast<const dpo_perception_subsystem*>(m_perception.get());
} /* dpo_perception() */

void dpo_controller::reset(void) {
  crw_controller::Reset();
  m_perception->reset();
} /* reset() */

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, goal_acquired, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, entity_acquired_id, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, is_exploring_for_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, is_vectoring_to_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, acquisition_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, block_transport_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, acquisition_loc3D, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, vector_loc3D, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(dpo_controller, explore_loc3D, *m_fsm, const);

/*******************************************************************************
 * Block Transportation Metrics
 ******************************************************************************/
bool dpo_controller::is_phototaxiing_to_goal(bool include_ca) const {
  return m_fsm->is_phototaxiing_to_goal(include_ca);
} /* is_phototaxiing_to_goal() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(dpo_controller, "dpo_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(cognitive, d0, controller, fordyca);
