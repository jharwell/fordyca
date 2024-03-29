/**
 * \file crw_controller.cpp
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
#include "fordyca/controller/reactive/d0/crw_controller.hpp"

#include <fstream>

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/config/nest_config.hpp"
#include "cosm/spatial/strategy/blocks/drop/factory.hpp"
#include "cosm/spatial/strategy/nest/acq/factory.hpp"
#include "cosm/spatial/strategy/nest/exit/factory.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "fordyca/controller/config/foraging_controller_repository.hpp"
#include "fordyca/fsm/d0/crw_fsm.hpp"
#include "fordyca/strategy/explore/block_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, reactive, d0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
crw_controller::crw_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.reactive.d0.crw") {}

crw_controller::~crw_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void crw_controller::init(ticpp::Element& node) {
  foraging_controller::init(node);
  ndc_uuid_push();
  ER_INFO("Initializing...");

  config::foraging_controller_repository repo;
  repo.parse_all(node);

  if (!repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  const auto* strat_config = repo.config_get<fsconfig::strategy_config>();

  csfsm::fsm_params fsm_params{ saa(), inta_tracker(), nz_tracker() };
  auto strategy_params = fstrategy::strategy_params {
    .fsm = &fsm_params,
    .explore = &strat_config->blocks.explore,
    .bsel_matrix = nullptr,
    .csel_matrix = nullptr,
    .accessor = nullptr,
    .ledtaxis_target = rutils::color()
  };

  const auto* nest = repo.config_get<crepr::config::nest_config>();

  auto block_search_factory = fsexplore::block_factory();
  auto nest_acq_factory = csstrategy::nest::acq::factory();
  auto nest_exit_factory = csstrategy::nest::exit::factory();
  auto block_drop_factory = csstrategy::blocks::drop::factory();

  auto block_acq = block_search_factory.create(fsexplore::block_factory::kCRW,
                                        &strategy_params,
                                        rng());
  auto nest_acq =
      nest_acq_factory.create(strat_config->nest.acq.strategy,
                          &strat_config->nest.acq,
                          &fsm_params,
                          rng());
  auto nest_exit =
      nest_exit_factory.create(strat_config->nest.exit.strategy,
                               &strat_config->nest.exit,
                               &fsm_params,
                               rng());
  auto block_drop = block_drop_factory.create(strat_config->blocks.drop.strategy,
                                              &fsm_params,
                                              &strat_config->blocks.drop,
                                              rng());

  cffsm::strategy_set strategies = {
    .explore = std::move(block_acq),
    .nest_acq = std::move(nest_acq),
    .nest_exit = std::move(nest_exit),
    .block_drop = std::move(block_drop)
  };
  m_fsm = std::make_unique<fsm::d0::crw_fsm>(&fsm_params,
                                             std::move(strategies),
                                             nest->center,
                                             rng());

  /* Set CRW FSM supervision */
  supervisor()->supervisee_update(m_fsm.get());
  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void crw_controller::reset(void) {
  foraging_controller::reset();
  if (nullptr != m_fsm) {
    m_fsm->init();
  }
} /* reset() */

void crw_controller::control_step(void) {
  mdc_ts_update();
  ndc_uuid_push();
  ER_ASSERT(!(nullptr != block() && !block()->is_carried_by_robot()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->md()->robot_id().v());

  /*
   * Reset steering forces tracking so per-timestep visualizations are
   * correct. This can't be done when applying the steering forces because then
   * they are always 0 during loop function visualization.
   */
  saa()->steer_force2D().tracking_reset();

  /*
   * Run the FSM and apply steering forces if normal operation, otherwise handle
   * abnormal operation state.
   */
  supervisor()->run();

  /* Update block detection status for use in the loop functions */
  block_detect_status_update();

  ndc_uuid_pop();
} /* control_step() */

/*******************************************************************************
 * Goal Acquisition
 ******************************************************************************/
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, goal_acquired, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, entity_acquired_id, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, is_exploring_for_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, acquisition_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, block_transport_goal, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, acquisition_loc3D, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, vector_loc3D, *m_fsm, const);
RCPPSW_WRAP_DEF_OVERRIDE(crw_controller, explore_loc3D, *m_fsm, const);

/*******************************************************************************
 * Block Transportation
 ******************************************************************************/
bool crw_controller::is_phototaxiing_to_goal(bool include_ca) const {
  return m_fsm->is_phototaxiing_to_goal(include_ca);
} /* is_phototaxiing_to_goal() */

/*******************************************************************************
 * Block Carrying Controller
 ******************************************************************************/
const cssblocks::drop::base_drop* crw_controller::block_drop_strategy(void) const {
  return m_fsm->block_drop_strategy();
} /* block_drop_strategy() */

NS_END(reactive, d0, controller, fordyca);

#if defined(COSM_PAL_TARGET_ARGOS)
using namespace fcrd0; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(crw_controller, "crw_controller");

RCPPSW_WARNING_DISABLE_POP()

#endif
