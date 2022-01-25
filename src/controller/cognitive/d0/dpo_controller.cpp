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
#include "cosm/spatial/strategy/nest_acq/factory.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/ds/cell2D.hpp"

#include "fordyca/controller/config/block_sel/block_sel_matrix_config.hpp"
#include "fordyca/controller/config/d0/dpo_controller_repository.hpp"
#include "fordyca/strategy/config/strategy_config.hpp"
#include "fordyca/controller/cognitive/block_sel_matrix.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/strategy/explore/block_factory.hpp"
#include "fordyca/subsystem/perception/perception_subsystem_factory.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"

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
      m_fsm() {}

dpo_controller::~dpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_controller::fsm(std::unique_ptr<fsm::d0::dpo_fsm> fsm) {
  m_fsm = std::move(fsm);
} /* fsm() */

void dpo_controller::control_step(void) {
  mdc_ts_update();
  ndc_uuid_push();

  /*
   * Reset steering forces tracking so per-timestep visualizations are
   * correct. This can't be done when applying the steering forces because then
   * they are always 0 during loop function visualization.
   */
  saa()->steer_force2D().tracking_reset();

  ER_ASSERT(!(nullptr != block() && !block()->is_carried_by_robot()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());

  /* Update perception */
  perception()->update(nullptr);

  /*
   * Run the FSM and apply steering forces if normal operation, otherwise handle
   * abnormal operation state.
   */
  supervisor()->run();

  ndc_uuid_pop();
} /* control_step() */

void dpo_controller::init(ticpp::Element& node) {
  cognitive_controller::init(node);

  ndc_uuid_push();
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
  ndc_uuid_pop();
} /* init() */

void dpo_controller::shared_init(
    const config::d0::dpo_controller_repository& config_repo) {
  const auto* perception_config = config_repo.config_get<fspconfig::perception_config>();
  const auto* block_matrix =
      config_repo.config_get<config::block_sel::block_sel_matrix_config>();
  const auto* nest = config_repo.config_get<crepr::config::nest_config>();

  /* DPO perception subsystem */
  auto factory = fsperception::perception_subsystem_factory();
  perception(factory.create(perception_config->type,
                            perception_config));

  /* block selection matrix */
  m_block_sel_matrix =
      std::make_unique<class block_sel_matrix>(block_matrix, nest->center);
} /* shared_init() */

void dpo_controller::private_init(
    const config::d0::dpo_controller_repository& config_repo) {
  const auto* strat_config =
      config_repo.config_get<fsconfig::strategy_config>();

  csfsm::fsm_params fsm_params {
      saa(),
      inta_tracker(),
      nz_tracker(),
    };

  auto strategy_params = fstrategy::strategy_params{
    &fsm_params,
    nullptr,
    nullptr,
    nullptr,
    rutils::color()
    };
  fsm::fsm_ro_params fsm_ro_params = {
    .bsel_matrix = block_sel_matrix(),
    .csel_matrix = nullptr,
    .store = perception()->model<fspds::dpo_store>(),
    .accessor = perception()->known_objects(),
    .strategy_config = *strat_config
  };
  m_fsm = std::make_unique<fsm::d0::dpo_fsm>(
      &fsm_ro_params,
      &fsm_params,
      fsexplore::block_factory().create(strat_config->explore.block_strategy,
                                        &strategy_params,
                                        rng()),
      csstrategy::nest_acq::factory().create(strat_config->nest_acq.strategy,
                                             &fsm_params,
                                             rng()),
      rng());

  /* Set DPO FSM supervision */
  supervisor()->supervisee_update(m_fsm.get());
} /* private_init() */

void dpo_controller::reset(void) {
  cognitive_controller::reset();
} /* reset() */

/*******************************************************************************
 * Goal Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller,
                          goal_acquired,
                          m_fsm,
                          false,
                          const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller,
                          entity_acquired_id,
                          m_fsm,
                          rtypes::constants::kNoUUID,
                          const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller,
                          is_exploring_for_goal,
                          m_fsm,
                          csmetrics::goal_acq_metrics::exp_status(),
                          const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller, is_vectoring_to_goal, m_fsm, false, const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller,
                          acquisition_goal,
                          m_fsm,
                          ffsm::to_goal_type(ffsm::foraging_acq_goal::ekNONE),
                          const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller, block_transport_goal,
                          m_fsm,
                          ffsm::foraging_transport_goal::ekNONE,
                          const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller,
                          acquisition_loc3D,
                          m_fsm,
                          rmath::vector3z(),
                          const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller,
                          vector_loc3D,
                          m_fsm,
                          rmath::vector3z(),
                          const);
RCPPSW_WRAP_DEFP_OVERRIDE(dpo_controller,
                          explore_loc3D,
                          m_fsm,
                          rmath::vector3z(),
                          const);

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
