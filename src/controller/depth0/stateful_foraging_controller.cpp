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
#include "fordyca/controller/depth0/stateful_foraging_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth0/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/params/depth0/stateful_foraging_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"
#include "rcppsw/task_allocation/task_decomposition_graph.hpp"
#include "rcppsw/task_allocation/task_params.hpp"
#include "rcppsw/task_allocation/executive_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_foraging_controller::stateful_foraging_controller(void)
    : stateless_foraging_controller(), m_light_loc(), m_executive() {}

stateful_foraging_controller::~stateful_foraging_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure std::shared_ptr<tasks::base_foraging_task> stateful_foraging_controller::
    current_task(void) const {
  return std::dynamic_pointer_cast<tasks::base_foraging_task>(
      m_executive->current_task());
} /* current_task() */

__rcsw_pure const representation::line_of_sight* stateful_foraging_controller::los(
    void) const {
  return stateful_sensors()->los();
}
void stateful_foraging_controller::los(
    std::unique_ptr<representation::line_of_sight>& new_los) {
  stateful_sensors()->los(new_los);
}

__rcsw_pure const std::shared_ptr<const depth0::sensing_subsystem>
stateful_foraging_controller::stateful_sensors(void) const {
  return std::static_pointer_cast<const depth0::sensing_subsystem>(
      saa_subsystem()->sensing());
}

__rcsw_pure std::shared_ptr<depth0::sensing_subsystem> stateful_foraging_controller::
    stateful_sensors(void) {
  return std::static_pointer_cast<depth0::sensing_subsystem>(
      saa_subsystem()->sensing());
}

void stateful_foraging_controller::ControlStep(void) {
  /*
   * Update the robot's model of the world with the current line-of-sight, and
   * update the relevance of information within it. Then, you can run the main
   * FSM loop.
   */
  m_perception->update(stateful_sensors()->los());

  saa_subsystem()->actuation()->block_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->block_throttle_update();

  m_executive->run();
} /* ControlStep() */

void stateful_foraging_controller::Init(ticpp::Element& node) {
  params::depth0::stateful_foraging_repository param_repo(server_ref());

  /*
   * Note that we do not call \ref stateless_foraging_controller::Init()--there
   * is nothing in there that we need.
   */
  base_foraging_controller::Init(node);

  ER_NOM("Initializing stateful_foraging controller");

  /* parse and validate parameters */
  param_repo.parse_all(node);
  server_handle()->log_stream() << param_repo;
  ER_ASSERT(param_repo.validate_all(),
            "FATAL: Not all parameters were validated");

  /* initialize subsystems and perception */
  m_perception = rcppsw::make_unique<base_perception_subsystem>(
      client::server_ref(),
      param_repo.parse_results<params::perception_params>(),
      GetId());

  saa_subsystem()->sensing(std::make_shared<depth0::sensing_subsystem>(
      param_repo.parse_results<struct params::sensing_params>(),
      &saa_subsystem()->sensing()->sensor_list()));

  /* initialize tasking */
  tasking_init(
      param_repo.parse_results<struct params::fsm_params>(),
      param_repo.parse_results<ta::executive_params>());

  ER_NOM("stateful_foraging controller initialization finished");
} /* Init() */

void stateful_foraging_controller::tasking_init(
    const struct params::fsm_params* fsm_params,
    const ta::executive_params* exec_params) {
  std::unique_ptr<ta::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_foraging_fsm>(
          fsm_params,
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
          m_perception->map());
  auto generalist =
      std::make_shared<tasks::depth0::generalist>(exec_params, generalist_fsm);

  generalist->set_atomic();

  auto graph = std::make_shared<ta::task_decomposition_graph>(server_ref());
  graph->set_root(generalist);

  m_executive = rcppsw::make_unique<ta::polled_executive>(server_ref(), graph);
} /* tasking_init() */

FSM_WRAPPER_DEFINE_PTR(transport_goal_type,
                       stateful_foraging_controller,
                       block_transport_goal,
                       current_task());

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
FSM_WRAPPER_DEFINE_PTR(bool,
                       stateful_foraging_controller,
                       is_avoiding_collision,
                       current_task());
FSM_WRAPPER_DEFINE_PTR(bool,
                       stateful_foraging_controller,
                       is_exploring_for_goal,
                       current_task());

FSM_WRAPPER_DEFINE_PTR(bool,
                       stateful_foraging_controller,
                       is_vectoring_to_goal,
                       current_task());

FSM_WRAPPER_DEFINE_PTR(bool,
                       stateful_foraging_controller,
                       goal_acquired,
                       current_task());

FSM_WRAPPER_DEFINE_PTR(acquisition_goal_type,
                       stateful_foraging_controller,
                       acquisition_goal,
                       current_task());

using namespace argos;
REGISTER_CONTROLLER(stateful_foraging_controller,
                    "stateful_foraging_controller"); // NOLINT

NS_END(depth0, controller, fordyca);
