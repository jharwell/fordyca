/**
 * @file depth1_controller.cpp
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
#include "fordyca/controller/depth1_controller.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/params/task_repository.hpp"
#include "rcppsw/task_allocation/task_params.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/fsm/block_to_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_controller::ControlStep(void) {
  /*
   * Update the perceived arena map with the current line-of-sight, and update
   * the relevance of information within it. Then, you can run the main FSM
   * loop.
   */
  memory_foraging_controller::map()->event_new_los(
      base_foraging_controller::sensors()->los());
  memory_foraging_controller::map()->update_density();
} /* ControlStep() */

void depth1_controller::Init(argos::TConfigurationNode& node) {
  params::task_repository task_repo;
  params::task_repository fsm_repo;

  memory_foraging_controller::Init(node);
  task_repo.parse_all(node);
  task_repo.show_all(server_handle()->log_stream());
  fsm_repo.parse_all(node);
  fsm_repo.show_all(server_handle()->log_stream());

  ER_NOM("Initializing depth1 controller");
  const task_allocation::task_params* p =
      static_cast<const task_allocation::task_params*>(
          task_repo.get_params("task"));

  std::unique_ptr<task_allocation::taskable> collector_fsm =
      rcppsw::make_unique<fsm::block_to_nest_fsm>(
          static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          base_foraging_controller::sensors(),
          base_foraging_controller::actuators(),
          memory_foraging_controller::map_ref());
  m_collector.reset(new tasks::collector(p->estimation_alpha,
                                         collector_fsm));
  m_collector->set_atomic();

  std::unique_ptr<task_allocation::taskable> forager_
      rcppsw::make_unique<fsm::block_to_cache_fsm>(
          static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          base_foraging_controller::sensors(),
          base_foraging_controller::actuators(),
          memory_foraging_controller::map_ref());
  m_forager.reset(new tasks::forager(p->estimation_alpha, forager_fsm));
  m_forager->set_atomic();

  /* std::unique_ptr<task_allocation::taskable> generalist_fsm = */
  /*     rcppsw::make_unique<fsm::memory_foraging_fsm>( */
  /*         static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")), */
  /*         base_foraging_controller::server(), */
  /*         base_foraging_controller::sensors(), */
  /*         base_foraging_controller::actuators(), */
  /*         memory_foraging_controller::map_ref()); */
  m_generalist.reset(new tasks::generalist(p, collector_fsm));

  ER_NOM("depth1 controller initialization finished");
} /* Init() */

using namespace argos;
REGISTER_CONTROLLER(depth1_controller, "depth1_controller")

NS_END(controller, fordyca);
