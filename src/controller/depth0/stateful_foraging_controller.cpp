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
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <fstream>

#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/events/block_found.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/params/depth0/perceived_arena_map_params.hpp"
#include "fordyca/params/depth0/stateful_foraging_repository.hpp"
#include "fordyca/params/depth1/task_params.hpp"
#include "fordyca/params/depth1/task_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensor_params.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cache.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/generalist.hpp"
#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"
#include "rcppsw/task_allocation/task_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_foraging_controller::stateful_foraging_controller(void)
    : stateless_foraging_controller(),
      m_light_loc(),
      m_map(),
      m_executive(),
      m_generalist() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__pure tasks::foraging_task* stateful_foraging_controller::current_task(
    void) const {
  return dynamic_cast<tasks::foraging_task*>(m_executive->current_task());
} /* current_task() */

bool stateful_foraging_controller::block_acquired(void) const {
  if (nullptr != current_task()) {
    return current_task()->block_acquired();
  }
  return false;
} /* block_acquired() */

__pure const representation::line_of_sight* stateful_foraging_controller::los(
    void) const {
  return stateful_sensors()->los();
}
void stateful_foraging_controller::los(
    std::unique_ptr<representation::line_of_sight>& new_los) {
  stateful_sensors()->los(new_los);
}

__pure depth1::foraging_sensors* stateful_foraging_controller::stateful_sensors(
    void) const {
  return static_cast<depth1::foraging_sensors*>(base_sensors());
}

std::shared_ptr<depth1::foraging_sensors> stateful_foraging_controller::
    stateful_sensors_ref(void) const {
  return std::static_pointer_cast<depth1::foraging_sensors>(base_sensors_ref());
}
void stateful_foraging_controller::ControlStep(void) {
  /*
   * Update the perceived arena map with the current line-of-sight, and update
   * the relevance of information within it. Then, you can run the main FSM
   * loop.
   */
  process_los(stateful_sensors()->los());
  m_map->update_density();

  if (is_carrying_block()) {
    actuators()->set_speed_throttle(true);
  } else {
    actuators()->set_speed_throttle(false);
  }

  m_executive->run();
} /* ControlStep() */

void stateful_foraging_controller::Init(argos::TConfigurationNode& node) {
  params::depth0::stateful_foraging_repository param_repo;
  params::depth1::task_repository task_repo;

  /*
   * Note that we do not call the stateless_foraging_controller::Init()--there
   * is nothing in there that we need.
   */
  base_foraging_controller::Init(node);

  ER_NOM("Initializing stateful_foraging controller");
  param_repo.parse_all(node);
  task_repo.parse_all(node);
  param_repo.show_all(server_handle()->log_stream());
  task_repo.show_all(server_handle()->log_stream());
  ER_ASSERT(param_repo.validate_all(),
            "FATAL: Not all parameters were validated");
  ER_ASSERT(task_repo.validate_all(),
            "FATAL: Not all task parameters were validated");

  m_map = rcppsw::make_unique<representation::perceived_arena_map>(
      server(),
      static_cast<const struct params::depth0::perceived_arena_map_params*>(
          param_repo.get_params("perceived_arena_map")),
      GetId());

  base_sensors(rcppsw::make_unique<depth1::foraging_sensors>(
      static_cast<const struct params::sensor_params*>(
          param_repo.get_params("sensors")),
      GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
      GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
      GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
      GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")));

  const params::fsm_params* fsm_params =
      static_cast<const struct params::fsm_params*>(
          param_repo.get_params("fsm"));

  const params::depth1::task_params* task_params =
      static_cast<const params::depth1::task_params*>(
          task_repo.get_params("task"));

  std::unique_ptr<task_allocation::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_foraging_fsm>(
          fsm_params,
          base_foraging_controller::server(),
          stateful_sensors_ref(),
          base_foraging_controller::actuators(),
          depth0::stateful_foraging_controller::map_ref());
  m_generalist = rcppsw::make_unique<tasks::generalist>(&task_params->tasks,
                                                        generalist_fsm);
  m_generalist->parent(m_generalist.get());
  m_generalist->set_atomic();

  m_executive = rcppsw::make_unique<task_allocation::polled_executive>(
      base_foraging_controller::server(), m_generalist.get());

  ER_NOM("stateful_foraging controller initialization finished");
} /* Init() */

void stateful_foraging_controller::process_los(
    const representation::line_of_sight* const los) {
  /*
   * If the robot thinks that a cell contains a block, because the cell had one
   * the last time it passed nearby, but when coming near the cell a second time
   * the cell does not contain a block, then someone else picked up the block
   * between then and now, and it needs to update its internal representation
   * accordingly.
   */
  for (size_t i = 0; i < los->xsize(); ++i) {
    for (size_t j = 0; j < los->ysize(); ++j) {
      representation::discrete_coord d = los->cell(i, j).loc();
      if (!los->cell(i, j).state_has_block() &&
          map()->access(d).state_has_block()) {
        ER_DIAG("Correct block%d discrepency at (%zu, %zu)",
                map()->access(d).block()->id(),
                d.first,
                d.second);
        map()->block_remove(map()->access(d).block());
      }
    } /* for(j..) */
  }   /* for(i..) */

  for (auto block : los->blocks()) {
    if (!m_map->access(block->discrete_loc()).state_has_block()) {
      ER_NOM("Discovered block%d at (%zu, %zu)",
             block->id(),
             block->discrete_loc().first,
             block->discrete_loc().second);
    }
    events::block_found op(base_foraging_controller::server(), block->clone());
    m_map->accept(op);
  } /* for(block..) */
} /* process_los() */

bool stateful_foraging_controller::is_transporting_to_nest(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_transporting_to_nest();
  }
  return false;
} /* is_transporting_to_nest() */

using namespace argos;
REGISTER_CONTROLLER(stateful_foraging_controller,
                    "stateful_foraging_controller"); // NOLINT

NS_END(depth0, controller, fordyca);
