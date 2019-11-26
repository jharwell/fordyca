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
#include "fordyca/controller/depth0/mdpo_controller.hpp"

#include "fordyca/config/depth0/mdpo_controller_repository.hpp"
#include "fordyca/config/exploration_config.hpp"
#include "fordyca/config/perception/perception_config.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/expstrat/block_factory.hpp"

#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_controller::mdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth0.mdpo") {}

mdpo_controller::~mdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void mdpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() && -1 == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id(),
            block()->robot_id());
  perception()->update(nullptr);
  saa()->steer_force2D_apply();
  fsm()->run();
  ndc_pop();
} /* control_step() */

void mdpo_controller::init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref crw_controller::init()--there
   * is nothing in there that we need.
   */
  base_controller::init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  config::depth0::mdpo_controller_repository config_repo;
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

void mdpo_controller::shared_init(
    const config::depth0::mdpo_controller_repository& config_repo) {
  /* block selection matrix and DPO subsystem */
  dpo_controller::shared_init(config_repo);

  /* MDPO perception subsystem */
  config::perception::perception_config p =
      *config_repo.config_get<config::perception::perception_config>();
  p.occupancy_grid.upper.x(p.occupancy_grid.upper.x() + 1);
  p.occupancy_grid.upper.y(p.occupancy_grid.upper.y() + 1);

  dpo_controller::perception(
      std::make_unique<mdpo_perception_subsystem>(&p, GetId()));
} /* shared_init() */

void mdpo_controller::private_init(
    const config::depth0::mdpo_controller_repository& config_repo) {
  auto* exp_config = config_repo.config_get<config::exploration_config>();
  fsm::expstrat::block_factory f;
  fsm::expstrat::foraging_expstrat::params expstrat_params(
      saa(), nullptr, nullptr, perception()->dpo_store());
  fsm::fsm_ro_params fsm_ro_params = {.bsel_matrix = block_sel_matrix(),
                                      .csel_matrix = nullptr,
                                      .store = perception()->dpo_store(),
                                      .exp_config = *exp_config};
  dpo_controller::fsm(std::make_unique<fsm::depth0::dpo_fsm>(
      &fsm_ro_params,
      saa(),
      f.create(exp_config->block_strategy, &expstrat_params, rng()),
      rng()));
} /* private_init() */

mdpo_perception_subsystem* mdpo_controller::mdpo_perception(void) {
  return static_cast<mdpo_perception_subsystem*>(dpo_controller::perception());
} /* perception() */

const mdpo_perception_subsystem* mdpo_controller::mdpo_perception(void) const {
  return static_cast<const mdpo_perception_subsystem*>(
      dpo_controller::perception());
} /* perception() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(mdpo_controller, "mdpo_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth0, controller, fordyca);
