/**
 * \file bitd_mdpo_controller.cpp
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
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"

#include "cosm/foraging/repr/base_cache.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"

#include "fordyca/config/depth1/controller_repository.hpp"
#include "fordyca/config/perception/perception_config.hpp"
#include "fordyca/controller/depth1/task_executive_builder.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bitd_mdpo_controller::bitd_mdpo_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.depth1.bitd_mdpo") {}

bitd_mdpo_controller::~bitd_mdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bitd_mdpo_controller::init(ticpp::Element& node) {
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
  ER_INFO("Initialization finished");
  ndc_pop();
} /* init() */

void bitd_mdpo_controller::control_step(void) {
  ndc_pusht();
  ER_ASSERT(!(nullptr != block() &&
              rtypes::constants::kNoUUID == block()->robot_id()),
            "Carried block%d has robot id=%d",
            block()->id().v(),
            block()->robot_id().v());

  perception()->update(nullptr);
  executive()->run();
  saa()->steer_force2D_apply();
  ndc_pop();
} /* control_step() */

void bitd_mdpo_controller::shared_init(
    const config::depth1::controller_repository& config_repo) {
  /* block/cache selection matrices, executive  */
  bitd_dpo_controller::shared_init(config_repo);

  /* MDPO perception subsystem */
  config::perception::perception_config p =
      *config_repo.config_get<config::perception::perception_config>();
  p.occupancy_grid.upper.x(p.occupancy_grid.upper.x() + 1);
  p.occupancy_grid.upper.y(p.occupancy_grid.upper.y() + 1);

  bitd_dpo_controller::perception(
      std::make_unique<mdpo_perception_subsystem>(&p, GetId()));

  /*
   * Task executive. Even though we use the same executive as the \ref
   * bitd_dpo_controller, we have to replace it because we have our own perception
   * subsystem, which is used to create the executive's graph.
   */
  executive(task_executive_builder(block_sel_matrix(),
                                   cache_sel_matrix(),
                                   saa(),
                                   perception())(config_repo, rng()));
  executive()->task_abort_notify(std::bind(
      &bitd_mdpo_controller::task_abort_cb, this, std::placeholders::_1));

} /* shared_init() */

mdpo_perception_subsystem* bitd_mdpo_controller::mdpo_perception(void) {
  return static_cast<mdpo_perception_subsystem*>(dpo_controller::perception());
} /* perception() */

const mdpo_perception_subsystem* bitd_mdpo_controller::mdpo_perception(
    void) const {
  return static_cast<const mdpo_perception_subsystem*>(
      dpo_controller::perception());
} /* perception() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_CONTROLLER(bitd_mdpo_controller, "bitd_mdpo_controller");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth1, controller, fordyca);
