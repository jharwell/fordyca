/**
 * @file memory_foraging_controller.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/memory_foraging_controller.hpp"
#include "fordyca/params/memory_foraging_repository.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/perceived_grid_params.hpp"
#include "fordyca/controller/sensor_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void memory_foraging_controller::robot_loc(argos::CVector2 loc) {
  sensors()->robot_loc(loc);
}
argos::CVector2 memory_foraging_controller::robot_loc(void) const {
  return sensors()->robot_loc();
}
const representation::line_of_sight* memory_foraging_controller::los(void) const {
  return sensors()->los();
}
void memory_foraging_controller::los(
    std::unique_ptr<representation::line_of_sight>& new_los) {
  sensors()->los(new_los);
}
void memory_foraging_controller::ControlStep(void) {
  /*
   * Update the perceived arena map with the current line-of-sight, and update
   * the relevance of information within it. Then, you can run the main FSM
   * loop.
   */
  m_map->event_new_los(sensors()->los());
  m_map->update_density();
  if (m_fsm->task_finished()) {
    m_fsm->task_reset();
    m_fsm->task_start(nullptr);
  } else {
    m_fsm->task_execute();
  }
} /* ControlStep() */

void memory_foraging_controller::Init(argos::TConfigurationNode& node) {
  params::memory_foraging_repository param_repo;

  base_foraging_controller::Init(node);
  ER_NOM("Initializing memory_foraging controller");
  param_repo.parse_all(node);
  param_repo.show_all(server_handle()->log_stream());

  m_map.reset(new representation::perceived_arena_map(
      server(),
      static_cast<const struct params::perceived_grid_params*>(
          param_repo.get_params("perceived_grid")),
      GetId()));

  m_fsm.reset(
      new fsm::memory_foraging_fsm(static_cast<const struct params::fsm_params*>(
          param_repo.get_params("fsm")),
                       server(),
                       sensors(),
                       actuators(),
                       m_map));
  ER_NOM("memory_foraging controller initialization finished");
} /* Init() */

using namespace argos;
REGISTER_CONTROLLER(memory_foraging_controller, "memory_foraging_controller")

NS_END(controller, fordyca);
