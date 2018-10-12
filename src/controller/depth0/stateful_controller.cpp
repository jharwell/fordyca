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
#include "fordyca/controller/block_selection_matrix.hpp"
#include "fordyca/controller/depth0/sensing_subsystem.hpp"
#include "fordyca/controller/depth0/stateful_tasking_initializer.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/depth0/stateful_controller_repository.hpp"
#include "fordyca/params/sensing_params.hpp"

#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_controller::stateful_controller(void)
    : stateless_controller(),
      ER_CLIENT_INIT("fordyca.controller.depth0.stateful"),
      m_light_loc(),
      m_block_sel_matrix(),
      m_perception(),
      m_executive() {}

stateful_controller::~stateful_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure const ta::bi_tab* stateful_controller::active_tab(
    void) const {
  return m_executive->active_tab();
}

void stateful_controller::block_sel_matrix(
    std::unique_ptr<block_selection_matrix> m) {
  m_block_sel_matrix = std::move(m);
}
void stateful_controller::executive(
    std::unique_ptr<ta::bi_tdgraph_executive> executive) {
  m_executive = std::move(executive);
}

void stateful_controller::perception(
    std::unique_ptr<base_perception_subsystem> perception) {
  m_perception = std::move(perception);
}
__rcsw_pure tasks::base_foraging_task* stateful_controller::current_task(void) {
  return dynamic_cast<tasks::base_foraging_task*>(
      m_executive.get()->current_task());
} /* current_task() */

__rcsw_pure const tasks::base_foraging_task* stateful_controller::current_task(
    void) const {
  return const_cast<stateful_controller*>(this)->current_task();
} /* current_task() */

__rcsw_pure const representation::line_of_sight* stateful_controller::los(
    void) const {
  return stateful_sensors()->los();
}
void stateful_controller::los(
    std::unique_ptr<representation::line_of_sight>& new_los) {
  stateful_sensors()->los(new_los);
}

__rcsw_pure const depth0::sensing_subsystem* stateful_controller::stateful_sensors(
    void) const {
  return static_cast<const depth0::sensing_subsystem*>(
      saa_subsystem()->sensing().get());
}

__rcsw_pure depth0::sensing_subsystem* stateful_controller::stateful_sensors(
    void) {
  return static_cast<depth0::sensing_subsystem*>(
      saa_subsystem()->sensing().get());
}

void stateful_controller::ControlStep(void) {
  ndc_pusht();
  /*
   * Update the robot's model of the world with the current line-of-sight, and
   * update the relevance of information within it. Then, you can run the main
   * FSM loop.
   */
  m_perception->update(stateful_sensors()->los());

  saa_subsystem()->actuation()->block_carry_throttle(is_carrying_block());
  saa_subsystem()->actuation()->throttling_update(stateful_sensors()->tick());
  m_executive->run();
  ndc_pop();
} /* ControlStep() */

void stateful_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref stateless_controller::Init()--there
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

  saa_subsystem()->sensing(std::make_shared<depth0::sensing_subsystem>(
      param_repo.parse_results<struct params::sensing_params>(),
      &saa_subsystem()->sensing()->sensor_list()));

  auto* ogrid = param_repo.parse_results<params::occupancy_grid_params>();
  m_block_sel_matrix =
      rcppsw::make_unique<block_selection_matrix>(ogrid->nest,
                                                  &ogrid->priorities);

  /* initialize tasking */
  m_executive = stateful_tasking_initializer(m_block_sel_matrix.get(),
                                             saa_subsystem(),
                                             perception())(&param_repo);
  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void stateful_controller::Reset(void) {
  stateless_controller::Reset();
  m_perception->reset();
} /* Reset() */

FSM_WRAPPER_DEFINE_PTR(transport_goal_type,
                       stateful_controller,
                       block_transport_goal,
                       current_task());

FSM_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                       stateful_controller,
                       acquisition_goal,
                       current_task());

FSM_WRAPPER_DEFINE_PTR(bool, stateful_controller, goal_acquired, current_task());

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
