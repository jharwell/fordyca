/**
 * \file mdpo_controller.cpp
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
#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/strategy/nest_acq/factory.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "fordyca/controller/config/d0/mdpo_controller_repository.hpp"
#include "fordyca/strategy/config/strategy_config.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/strategy/explore/block_factory.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/perception_subsystem_factory.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_controller::mdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.d0.mdpo") {}

mdpo_controller::~mdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void mdpo_controller::control_step(void) {
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

  perception()->update(nullptr);

  /*
   * Run the FSM and apply steering forces if normal operation, otherwise handle
   * abnormal operation state.
   */
  supervisor()->run();

  ndc_uuid_pop();
} /* control_step() */

void mdpo_controller::init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref crw_controller::init()--there
   * is nothing in there that we need.
   */
  foraging_controller::init(node);

  ndc_uuid_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  config::d0::mdpo_controller_repository config_repo;
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

void mdpo_controller::shared_init(
    const config::d0::mdpo_controller_repository& config_repo) {
  /* block selection matrix and DPO subsystem */
  dpo_controller::shared_init(config_repo);

  /* MDPO perception subsystem */
  auto p = *config_repo.config_get<fspconfig::perception_config>();
  rmath::vector2d padding(p.mdpo.rlos.grid2D.resolution.v() * 5,
                          p.mdpo.rlos.grid2D.resolution.v() * 5);
  p.mdpo.rlos.grid2D.dims += padding;
  auto factory = fsperception::perception_subsystem_factory();
  perception(factory.create(p.type, &p));
} /* shared_init() */

void mdpo_controller::private_init(
    const config::d0::mdpo_controller_repository& config_repo) {
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
    perception()->known_objects(),
    rutils::color()
  };
  fsm::fsm_ro_params fsm_ro_params = {
    .bsel_matrix = block_sel_matrix(),
    .csel_matrix = nullptr,
    .store = perception()->model<fspds::dpo_semantic_map>()->store(),
    .accessor = perception()->known_objects(),
    .strategy_config = *strat_config };
  dpo_controller::fsm(std::make_unique<fsm::d0::dpo_fsm>(
      &fsm_ro_params,
      &fsm_params,
      fsexplore::block_factory().create(strat_config->explore.block_strategy,
                                        &strategy_params,
                                        rng()),
      csstrategy::nest_acq::factory().create(strat_config->nest_acq.strategy,
                                             &fsm_params,
                                             rng()),
      rng()));

  /* Set MDPO FSM supervision */
  supervisor()->supervisee_update(fsm());
} /* private_init() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(mdpo_controller, "mdpo_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(cognitive, d0, controller, fordyca);
