/**
 * @file foraging_controller.cpp
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
#include "fordyca/controller/depth2/foraging_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/perception_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/depth2/param_repository.hpp"
#include "fordyca/params/sensing_params.hpp"

#include "fordyca/controller/depth2/tasking_initializer.hpp"
#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/partitionable_task.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller::foraging_controller(void)
    : depth1::foraging_controller(), m_executive() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_controller::ControlStep(void) {
  perception()->update(depth0::stateful_foraging_controller::los());

  saa_subsystem()->actuation()->block_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->block_throttle_update();

  m_executive->run();
} /* ControlStep() */

void foraging_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref depth1::foraging_controller::Init()--there
   * is nothing in there that we need.
   */
  base_foraging_controller::Init(node);

  ER_NOM("Initializing depth2 foraging controller");

  params::depth2::param_repository param_repo(client::server_ref());

  param_repo.parse_all(node);
  server_ptr()->log_stream() << param_repo;

  ER_ASSERT(param_repo.validate_all(),
            "FATAL: Not all stateful foraging parameters were validated");

  /* Put in new depth1 sensors and perception, ala strategy pattern */
  saa_subsystem()->sensing(std::make_shared<depth1::sensing_subsystem>(
      param_repo.parse_results<struct params::sensing_params>(),
      &saa_subsystem()->sensing()->sensor_list()));

  perception(rcppsw::make_unique<depth1::perception_subsystem>(
      client::server_ref(),
      param_repo.parse_results<params::perception_params>(),
      GetId()));

  /* initialize tasking */
  m_executive = tasking_initializer(client::server_ref(),
                                    saa_subsystem(),
                                    perception())(&param_repo);

  ER_NOM("depth2 foraging controller initialization finished");
} /* Init() */

__rcsw_pure tasks::base_foraging_task* foraging_controller::
    current_task(void) {
  return dynamic_cast<tasks::base_foraging_task*>(m_executive->current_task());
} /* current_task() */

__rcsw_pure const tasks::base_foraging_task* foraging_controller::current_task(void) const {
  return const_cast<foraging_controller*>(this)->current_task();
} /* current_task() */

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
using depth2_foraging_controller = foraging_controller;

REGISTER_CONTROLLER(depth2_foraging_controller,
                    "depth2_foraging_controller"); // NOLINT

NS_END(depth2, controller, fordyca);
