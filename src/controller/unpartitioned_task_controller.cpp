/**
 * @file unpartitioned_task_controller.cpp
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
#include "fordyca/controller/unpartitioned_task_controller.hpp"
#include "fordyca/params/unpartitioned_task_repository.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void unpartitioned_task_controller::ControlStep(void) {
  /*
   * Update the perceived arena map with the current line-of-sight, and update
   * the relevance of information within it.
   */
  m_map->event_new_los(sensors()->los());
  m_map->update_density();
  m_fsm->run();
} /* ControlStep() */

void unpartitioned_task_controller::publish_event(enum event_type type) {
  switch (type) {
    case BLOCK_FOUND:
      m_fsm->event_block_found();
      break;
    default:
      break;
  }
} /* publish_event() */

void unpartitioned_task_controller::pickup_block(representation::block* block) {
  random_foraging_controller::pickup_block(block);
  m_map->event_block_pickup(block);
} /* pickup_block() */

void unpartitioned_task_controller::Init(argos::TConfigurationNode& node) {
  params::unpartitioned_task_repository param_repo;

  random_foraging_controller::Init(node);
  ER_NOM("Initializing unpartitioned_task controller");
  param_repo.parse_all(node);
  param_repo.show_all(server_handle()->log_stream());

  m_map.reset(new representation::perceived_arena_map(
      server(),
      static_cast<const struct perceived_grid_params*>(
          param_repo.get_params("perceived_grid"))));

  m_fsm.reset(
      new unpartitioned_task_fsm(static_cast<const struct foraging_fsm_params*>(
          param_repo.get_params("fsm")),
                       server(),
                       sensors(),
                       actuators(),
                       m_map));
ER_NOM("unpartitioned_task controller initialization finished");
} /* Init() */

/*
 * This statement notifies ARGoS of the existence of the controller.  It binds
 * the class passed as first argument to the string passed as second argument,
 * which is then available in the XML.
 */
using namespace argos;
REGISTER_CONTROLLER(unpartitioned_task_controller, "unpartitioned_task_controller")

NS_END(controller, fordyca);
