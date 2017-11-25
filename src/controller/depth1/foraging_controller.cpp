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
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

#include "rcppsw/task_allocation/task_params.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/params/depth1/task_repository.hpp"
#include "fordyca/params/depth0/stateful_foraging_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensor_params.hpp"
#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_cache_fsm.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/tasks/collector.hpp"
#include "fordyca/tasks/forager.hpp"
#include "fordyca/tasks/generalist.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller::foraging_controller(void) :
    depth0::stateful_foraging_controller(),
    m_task_aborted(false),
    m_executive(),
    m_forager(),
    m_collector(),
    m_generalist() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_controller::ControlStep(void) {
  /*
   * Update the perceived arena map with the current line-of-sight, and update
   * the relevance of information within it. Then, you can run the main FSM
   * loop.
   */
  process_los(depth0::stateful_foraging_controller::los());
  depth0::stateful_foraging_controller::map()->update_density();
  m_task_aborted = false;
  m_executive->run();
} /* ControlStep() */

void foraging_controller::Init(argos::TConfigurationNode& node) {
  params::depth1::task_repository task_repo;
  params::depth0::stateful_foraging_repository fsm_repo;

  depth0::stateful_foraging_controller::Init(node);
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
          depth0::stateful_foraging_controller::sensors_ref(),
          base_foraging_controller::actuators(),
          depth0::stateful_foraging_controller::map_ref());
  m_collector.reset(new tasks::collector(p, collector_fsm));

  std::unique_ptr<task_allocation::taskable> forager_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_cache_fsm>(
          static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          depth0::stateful_foraging_controller::sensors_ref(),
          base_foraging_controller::actuators(),
          depth0::stateful_foraging_controller::map_ref());
  m_forager.reset(new tasks::forager(p, forager_fsm));

  std::unique_ptr<task_allocation::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_foraging_fsm>(
          static_cast<const params::fsm_params*>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          depth0::stateful_foraging_controller::sensors_ref(),
          base_foraging_controller::actuators(),
          depth0::stateful_foraging_controller::map_ref());
  m_generalist.reset(new tasks::generalist(p, generalist_fsm));

  m_generalist->partition1(m_forager.get());
  m_generalist->partition2(m_collector.get());
  m_generalist->parent(m_generalist.get());
  m_generalist->set_partitionable();

  m_forager->parent(m_generalist.get());
  m_collector->parent(m_generalist.get());

  m_executive.reset(new task_allocation::polled_executive(
      base_foraging_controller::server(),
      m_generalist.get()));

  m_executive->task_abort_cleanup(std::bind(
      &foraging_controller::task_abort_cleanup,
      this,
      std::placeholders::_1));

  ER_NOM("depth1 controller initialization finished");
} /* Init() */

void foraging_controller::task_abort_cleanup(task_allocation::executable_task* const) {
  m_task_aborted = true;
} /* task_abort_cleanup() */

tasks::foraging_task* foraging_controller::current_task(void) const {
  return dynamic_cast<tasks::foraging_task*>(m_executive->current_task());
} /* current_task() */

bool foraging_controller::cache_acquired(void) const {
  if (current_task()) {
    return current_task()->cache_acquired();
  } else {
    return false;
  }
} /* cache_detected() */

bool foraging_controller::block_acquired(void) const {
  if (current_task()) {
    return current_task()->block_acquired();
  } else {
    return false;
  }
} /* block_detected() */

void foraging_controller::process_los(const representation::line_of_sight* const los) {
  depth0::stateful_foraging_controller::process_los(los);

  for (auto cache : los->caches()) {
    if (!map()->access(cache->discrete_loc().first,
                       cache->discrete_loc().second).state_has_cache()) {
      events::cache_found op(base_foraging_controller::server(), cache,
                             cache->discrete_loc().first,
                             cache->discrete_loc().second);
      map()->accept(op);
      ER_NOM("Discovered cache%d at (%zu, %zu)", cache->id(),
             cache->discrete_loc().first, cache->discrete_loc().second);
    }
  } /* for(cache..) */
} /* process_los() */

/*******************************************************************************
 * Distance Diagnostics
 ******************************************************************************/
double foraging_controller::timestep_distance(void) const {
  /*
   * If you allow distance gathering at timesteps <= 2, you get a big jump
   * because of the prev/current location not being set up properly yet. Might
   * be worth fixing at some point...
   */
  if (sensors()->tick() > 2) {
    return sensors()->robot_heading().Length();
  }
  return 0;
} /* timestep_distance() */

/*******************************************************************************
 * Stateless Diagnostics
 ******************************************************************************/
bool foraging_controller::is_exploring_for_block(void) const {
  if (current_task()) {
    return current_task()->is_exploring_for_block();
  } else {
    return false;
  }
  } /* is_exploring_for_block() */

bool foraging_controller::is_avoiding_collision(void) const {
  if (current_task()) {
  return current_task()->is_avoiding_collision();
  } else {
    return false;
  }
} /* is_avoiding_collision() */

bool foraging_controller::is_transporting_to_nest(void) const {
  if (current_task()) {
    return current_task()->is_transporting_to_nest();
  } else {
    return false;
  }
} /* is_transporting_to_nest() */

/*******************************************************************************
 * Stateful Diagnostics
 ******************************************************************************/
bool foraging_controller::is_acquiring_block(void) const {
  if (current_task()) {
    return current_task()->is_acquiring_block();
  } else {
    return false;
  }
} /* is_exploring() */

bool foraging_controller::is_vectoring_to_block(void) const {
  if (current_task()) {
  return current_task()->is_vectoring_to_block();
  } else {
    return false;
  }
} /* is_vectoring_to_block() */

/*******************************************************************************
 * Depth1 Diagnostics
 ******************************************************************************/
bool foraging_controller::is_exploring_for_cache(void) const {
  if (current_task()) {
    return current_task()->is_exploring_for_cache();
  } else {
    return false;
  }
} /* is_exploring_for_cache() */

bool foraging_controller::is_vectoring_to_cache(void) const {
  if (current_task()) {
    return current_task()->is_vectoring_to_cache();
  } else {
    return false;
  }
} /* is_vectoring_to_cache() */

bool foraging_controller::is_acquiring_cache(void) const {
  if (current_task()) {
    return current_task()->is_acquiring_cache();
  } else {
    return false;
  }
} /* is_acquring_to_cache() */

bool foraging_controller::is_transporting_to_cache(void) const {
  if (current_task()) {
    return current_task()->is_transporting_to_cache();
  } else {
    return false;
  }
} /* is_transporting_to_cache() */

std::string foraging_controller::task_name(void) const {
  if (current_task()) {
    return current_task()->task_name();
  } else {
    return "";
  }
} /* task_name() */

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
typedef foraging_controller depth1_foraging_controller;
REGISTER_CONTROLLER(depth1_foraging_controller, "depth1_foraging_controller");

NS_END(depth1, controller, fordyca);
