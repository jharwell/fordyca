/**
 * @file depth1_foraging_controller.cpp
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
#include "fordyca/controller/depth1_foraging_controller.hpp"
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

#include "rcppsw/task_allocation/task_params.hpp"
#include "fordyca/params/task_repository.hpp"
#include "fordyca/params/memory_foraging_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensor_params.hpp"
#include "fordyca/controller/depth1_foraging_sensors.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/fsm/block_to_cache_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_foraging_controller::ControlStep(void) {
  /*
   * Update the perceived arena map with the current line-of-sight, and update
   * the relevance of information within it. Then, you can run the main FSM
   * loop.
   */
  memory_foraging_controller::process_los(m_sensors->los());
  memory_foraging_controller::map()->update_density();

  m_executive->run();
} /* ControlStep() */

void depth1_foraging_controller::Init(argos::TConfigurationNode& node) {
  params::task_repository task_repo;
  params::memory_foraging_repository fsm_repo;

  memory_foraging_controller::Init(node);
  task_repo.parse_all(node);
  task_repo.show_all(server_handle()->log_stream());
  fsm_repo.parse_all(node);
  fsm_repo.show_all(server_handle()->log_stream());

  ER_NOM("Initializing depth1 controller");
  const task_allocation::task_params* p =
      static_cast<const task_allocation::task_params*>(
          task_repo.get_params("task"));

  m_sensors.reset(new depth1_foraging_sensors(
      static_cast<const struct params::sensor_params*>(
          fsm_repo.get_params("sensors")),
      GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
      GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
      GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
      GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")));

  std::unique_ptr<task_allocation::taskable> collector_fsm =
      rcppsw::make_unique<fsm::block_to_nest_fsm>(
          static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          m_sensors,
          base_foraging_controller::actuators(),
          memory_foraging_controller::map_ref());
  m_collector.reset(new tasks::collector(p->estimation_alpha,
                                         collector_fsm));
  m_collector->set_atomic();

  std::unique_ptr<task_allocation::taskable> forager_fsm =
      rcppsw::make_unique<fsm::block_to_cache_fsm>(
          static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          m_sensors,
          base_foraging_controller::actuators(),
          memory_foraging_controller::map_ref());
  m_forager.reset(new tasks::forager(p->estimation_alpha, forager_fsm));
  m_forager->set_atomic();

  std::unique_ptr<task_allocation::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::memory_foraging_fsm>(
          static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          m_sensors,
          base_foraging_controller::actuators(),
          memory_foraging_controller::map_ref());
  m_generalist.reset(new tasks::generalist(p, generalist_fsm));

  m_generalist->partition1(m_forager.get());
  m_generalist->partition2(m_collector.get());
  m_generalist->parent(m_generalist.get());

  m_forager->parent(m_generalist.get());
  m_collector->parent(m_generalist.get());

  m_executive.reset(new task_allocation::polled_executive(base_foraging_controller::server(),
                                                          m_generalist.get()));
  ER_NOM("depth1 controller initialization finished");
} /* Init() */

tasks::foraging_task* depth1_foraging_controller::current_task(void) const {
  return dynamic_cast<tasks::foraging_task*>(m_executive->current_task());
} /* current_task() */

bool depth1_foraging_controller::cache_detected(void) const {
  return m_sensors->cache_detected();
} /* cache_detected() */

/*******************************************************************************
 * Depth0 Diagnostics
 ******************************************************************************/
bool depth1_foraging_controller::is_searching_for_block(void) const {
  return current_task()->is_searching_for_block();
} /* is_searching_for_block() */

bool depth1_foraging_controller::is_avoiding_collision(void) const {
  return current_task()->is_avoiding_collision();
} /* is_avoiding_collision() */

bool depth1_foraging_controller::is_transporting_to_nest(void) const {
  return current_task()->is_transporting_to_nest();
} /* is_transporting_to_nest() */

bool depth1_foraging_controller::is_vectoring(void) const {
  return current_task()->is_vectoring();
} /* is_vectoring() */

bool depth1_foraging_controller::is_exploring(void) const {
  return current_task()->is_exploring();
} /* is_exploring() */

/*******************************************************************************
 * Depth1 Diagnostics
 ******************************************************************************/
bool depth1_foraging_controller::is_searching_for_cache(void) const {
  return current_task()->is_searching_for_cache();
} /* is_searching_for_cache() */

bool depth1_foraging_controller::is_transporting_to_cache(void) const {
  return current_task()->is_transporting_to_cache();
} /* is_transporting_to_cache() */

std::string depth1_foraging_controller::task_name(void) const {
  return current_task()->task_name();
} /* task_name() */

using namespace argos;
REGISTER_CONTROLLER(depth1_foraging_controller, "depth1_foraging_controller");

NS_END(controller, fordyca);
