/**
 * @file vectored_controller.cpp
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
#include "fordyca/controller/vectored_controller.hpp"
#include "fordyca/params/vectored_controller_repository.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void vectored_controller::ControlStep(void) {
  /*
   * Update the perceived arena map with the current line-of-sight, and update
   * the relevance of information within it.
   */
  m_map->event_new_los(sensors()->los());
  m_map->update_relevance();
  base_controller::ControlStep();
} /* ControlStep() */

void vectored_controller::pickup_block(representation::block* block) {
  base_controller::pickup_block(block);
  m_map->event_block_pickup(block);
} /* pickup_block() */

void vectored_controller::Init(argos::TConfigurationNode& node) {
  base_controller::Init(node);
  params::vectored_controller_repository param_repo;
  param_repo.parse_all(node);
  param_repo.show_all(server_handle()->log_stream());

  m_map.reset(new representation::perceived_arena_map(
      static_cast<const struct perceived_grid_params*>(
          param_repo.get_params("perceived_grid"))));
} /* Init() */

/*
 * This statement notifies ARGoS of the existence of the controller.  It binds
 * the class passed as first argument to the string passed as second argument,
 * which is then available in the XML.
 */
using namespace argos;
REGISTER_CONTROLLER(vectored_controller, "vectored_controller")

NS_END(controller, fordyca);
