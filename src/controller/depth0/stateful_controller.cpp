/**
 * @file stateful_controller.cpp
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
#include "fordyca/controller/depth0/stateful_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/fsm/depth0/stateful_fsm.hpp"
#include "fordyca/params/block_sel_matrix_params.hpp"
#include "fordyca/params/depth0/stateful_controller_repository.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "fordyca/representation/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_controller::stateful_controller(void)
    : crw_controller(),
      ER_CLIENT_INIT("fordyca.controller.depth0.stateful"),
      m_light_loc(),
      m_block_sel_matrix(),
      m_perception(),
      m_fsm() {}

stateful_controller::~stateful_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_controller::block_sel_matrix(
    std::unique_ptr<class block_sel_matrix> m) {
  m_block_sel_matrix = std::move(m);
}

void stateful_controller::perception(
    std::unique_ptr<base_perception_subsystem> perception) {
  m_perception = std::move(perception);
}

__rcsw_pure const representation::line_of_sight* stateful_controller::los(
    void) const {
  return m_perception->los();
}
void stateful_controller::los(
    std::unique_ptr<representation::line_of_sight>& new_los) {
  m_perception->los(new_los);
}

double stateful_controller::los_dim(void) const {
  return saa_subsystem()->sensing()->los_dim();
} /* los_dim() */

void stateful_controller::ControlStep(void) {
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
  m_perception->update(m_perception->los());

  saa_subsystem()->actuation()->motion_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->motion_throttle_update(
      saa_subsystem()->sensing()->tick());
  m_fsm->run();
  ndc_pop();
} /* ControlStep() */

void stateful_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref crw_controller::Init()--there
   * is nothing in there that we need.
   */
  base_controller::Init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* parse and validate parameters */
  params::depth0::stateful_controller_repository param_repo;
  param_repo.parse_all(node);

  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize subsystems and perception */
  m_perception = rcppsw::make_unique<base_perception_subsystem>(
      param_repo.parse_results<params::perception_params>(), GetId());

  auto* block_mat = param_repo.parse_results<params::block_sel_matrix_params>();
  m_block_sel_matrix = rcppsw::make_unique<class block_sel_matrix>(block_mat);

  m_fsm = rcppsw::make_unique<fsm::depth0::stateful_fsm>(
      m_block_sel_matrix.get(),
      base_controller::saa_subsystem(),
      m_perception->map());

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void stateful_controller::Reset(void) {
  crw_controller::Reset();
  m_perception->reset();
} /* Reset() */

FSM_WRAPPER_DEFINEC_PTR(transport_goal_type,
                        stateful_controller,
                        block_transport_goal,
                        m_fsm);

FSM_WRAPPER_DEFINEC_PTR(acquisition_goal_type,
                        stateful_controller,
                        acquisition_goal,
                        m_fsm);

FSM_WRAPPER_DEFINEC_PTR(bool, stateful_controller, goal_acquired, m_fsm);

/*******************************************************************************
 * World Model Metrics
 ******************************************************************************/
uint stateful_controller::cell_state_inaccuracies(uint state) const {
  return m_perception->cell_state_inaccuracies(state);
} /* cell_state_inaccuracies() */

double stateful_controller::known_percentage(void) const {
  return m_perception->known_percentage();
} /* known_percentage() */

double stateful_controller::unknown_percentage(void) const {
  return m_perception->unknown_percentage();
} /* unknown_percentage() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_CONTROLLER(stateful_controller, "stateful_controller");
#pragma clang diagnostic pop

NS_END(depth0, controller, fordyca);
