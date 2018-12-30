/**
 * @file mdpo_controller.cpp
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
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/params/depth0/mdpo_controller_repository.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/params/perception/perception_params.hpp"
#include "fordyca/params/block_sel_matrix_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_controller::mdpo_controller(void)
    : dpo_controller(),
      ER_CLIENT_INIT("fordyca.controller.depth0.mdpo") {}

mdpo_controller::~mdpo_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void mdpo_controller::ControlStep(void) {
  ndc_pusht();
  if (nullptr != block()) {
    ER_ASSERT(-1 != block()->robot_id(),
              "Carried block%d has robot id=%d",
              block()->id(),
              block()->robot_id());
  }

  /*
   * Update the robot's model of the world with the current line-of-sight, and
   * update the relevance of information within it. Then, you can run the main
   * FSM loop.
   */
  perception()->update();

  saa_subsystem()->actuation()->motion_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->motion_throttle_update(
      saa_subsystem()->sensing()->tick());
  fsm()->run();
  ndc_pop();
} /* ControlStep() */

void mdpo_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref crw_controller::Init()--there
   * is nothing in there that we need.
   */
  base_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  params::depth0::mdpo_controller_repository param_repo;
  param_repo.parse_all(node);

  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  shared_init(param_repo);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void mdpo_controller::shared_init(
    const params::depth0::mdpo_controller_repository& param_repo) {
  /* block selection matrix and DPO subsystem */
  dpo_controller::shared_init(param_repo);

  /* MDPO perception subsystem */
  params::perception::perception_params p =
      *param_repo.parse_results<params::perception::perception_params>();
  p.occupancy_grid.upper.x(p.occupancy_grid.upper.x() + 1);
  p.occupancy_grid.upper.y(p.occupancy_grid.upper.y() + 1);

  dpo_controller::perception(
      rcppsw::make_unique<mdpo_perception_subsystem>(&p, GetId()));
} /* shared_init() */

void mdpo_controller::private_init(void) {
  dpo_controller::fsm(rcppsw::make_unique<fsm::depth0::dpo_fsm>(
      block_sel_matrix(),
      base_controller::saa_subsystem(),
      &perception()->map()->store()));
} /* private_init() */

const mdpo_perception_subsystem* mdpo_controller::perception(void) const {
  return static_cast<const mdpo_perception_subsystem*>(dpo_controller::perception());
} /* perception() */

mdpo_perception_subsystem* mdpo_controller::perception(void) {
  return static_cast<mdpo_perception_subsystem*>(dpo_controller::perception());
} /* perception() */

/*******************************************************************************
 * World Model Metrics
 ******************************************************************************/
uint mdpo_controller::cell_state_inaccuracies(uint state) const {
  return static_cast<const mdpo_perception_subsystem*>(
      perception())->cell_state_inaccuracies(state);
} /* cell_state_inaccuracies() */

double mdpo_controller::known_percentage(void) const {
  return static_cast<const mdpo_perception_subsystem*>(
      perception())->known_percentage();
} /* known_percentage() */

double mdpo_controller::unknown_percentage(void) const {
  return static_cast<const mdpo_perception_subsystem*>(
      perception())->unknown_percentage();
} /* unknown_percentage() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(mdpo_controller, "mdpo_controller");
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
