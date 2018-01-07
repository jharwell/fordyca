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
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <fstream>

#include "fordyca/controller/depth1/foraging_sensors.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_cache_fsm.hpp"
#include "fordyca/params/depth0/stateful_foraging_repository.hpp"
#include "fordyca/params/depth1/task_params.hpp"
#include "fordyca/params/depth1/task_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensor_params.hpp"
#include "fordyca/representation/cache.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/collector.hpp"
#include "fordyca/tasks/forager.hpp"
#include "fordyca/tasks/generalist.hpp"
#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller::foraging_controller(void)
    : depth0::stateful_foraging_controller(),
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
  if (is_carrying_block()) {
    actuators()->set_speed_throttle(true);
  } else {
    actuators()->set_speed_throttle(false);
  }
  m_executive->run();
} /* ControlStep() */

void foraging_controller::Init(argos::TConfigurationNode &node) {
  params::depth1::task_repository task_repo;
  params::depth0::stateful_foraging_repository fsm_repo;

  depth0::stateful_foraging_controller::Init(node);
  task_repo.parse_all(node);
  task_repo.show_all(server_handle()->log_stream());
  fsm_repo.parse_all(node);
  fsm_repo.show_all(server_handle()->log_stream());

  ER_ASSERT(task_repo.validate_all(),
            "FATAL: Not all FSM parameters were validated");
  ER_ASSERT(fsm_repo.validate_all(),
            "FATAL: Not all task parameters were validated");

  ER_NOM("Initializing depth1 controller");
  const params::depth1::task_params *p =
      static_cast<const params::depth1::task_params *>(
          task_repo.get_params("task"));

  std::unique_ptr<task_allocation::taskable> collector_fsm =
      rcppsw::make_unique<fsm::block_to_nest_fsm>(
          static_cast<const params::fsm_params *>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          depth0::stateful_foraging_controller::stateful_sensors_ref(),
          base_foraging_controller::actuators(),
          depth0::stateful_foraging_controller::map_ref());
  m_collector = rcppsw::make_unique<tasks::collector>(&p->tasks, collector_fsm);

  std::unique_ptr<task_allocation::taskable> forager_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_cache_fsm>(
          static_cast<const params::fsm_params *>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          depth0::stateful_foraging_controller::stateful_sensors_ref(),
          base_foraging_controller::actuators(),
          depth0::stateful_foraging_controller::map_ref());
  m_forager = rcppsw::make_unique<tasks::forager>(&p->tasks, forager_fsm);

  std::unique_ptr<task_allocation::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_foraging_fsm>(
          static_cast<const params::fsm_params *>(fsm_repo.get_params("fsm")),
          base_foraging_controller::server(),
          depth0::stateful_foraging_controller::stateful_sensors_ref(),
          base_foraging_controller::actuators(),
          depth0::stateful_foraging_controller::map_ref());
  m_generalist = rcppsw::make_unique<tasks::generalist>(&p->tasks,
                                                        generalist_fsm);

  m_generalist->partition1(m_forager.get());
  m_generalist->partition2(m_collector.get());
  m_generalist->parent(m_generalist.get());
  m_generalist->set_partitionable();

  m_forager->parent(m_generalist.get());
  m_collector->parent(m_generalist.get());

  m_executive = rcppsw::make_unique<task_allocation::polled_executive>(
      base_foraging_controller::server(),
      m_generalist.get());

  m_executive->task_abort_cleanup(std::bind(
      &foraging_controller::task_abort_cleanup, this, std::placeholders::_1));

  if (p->init_random_estimates) {
    m_generalist->init_random(500, 1000);
    m_forager->init_random(100, 500);
    m_collector->init_random(100, 500);
  }
  ER_NOM("depth1 controller initialization finished");
} /* Init() */

void foraging_controller::task_abort_cleanup(
    task_allocation::executable_task *const) {
  m_task_aborted = true;
} /* task_abort_cleanup() */

__pure tasks::foraging_task *foraging_controller::current_task(void) const {
  return dynamic_cast<tasks::foraging_task *>(m_executive->current_task());
} /* current_task() */

bool foraging_controller::cache_acquired(void) const {
  if (nullptr !=current_task()) {
    return current_task()->cache_acquired();
  }
  return false;
} /* cache_detected() */

bool foraging_controller::block_acquired(void) const {
  if (nullptr != current_task()) {
    return current_task()->block_acquired();
  }
  return false;
} /* block_detected() */

void foraging_controller::process_los(
    const representation::line_of_sight *const c_los) {
  depth0::stateful_foraging_controller::process_los(c_los);

  for (auto cache : c_los->caches()) {
    /*
     * The state of a cache can change between when the robot saw it last
     * (i.e. different # of blocks in it), and so you need to always process
     * caches in the LOS, even if you already know about them.
     */
    if (!map()->access(cache->discrete_loc()).state_has_cache()) {
      ER_NOM("Discovered cache%d at (%zu, %zu)",
             cache->id(),
             cache->discrete_loc().first,
             cache->discrete_loc().second);
    }
    /*
     * The cache we get a handle to is owned by the simulation, and we don't
     * want to just pass that into the robot's arena_map, as keeping them in
     * sync is not possible in all situations.
     *
     * For example if a block executing the collector task picks up a block and
     * tries to compute the best cache to bring it to, only to have one or more
     * of its cache references be invalid due to other robots causing caches to
     * be created/destroyed.
     *
     * Cloning is definitely necessary here.
     */
    std::unique_ptr<representation::cache> clone = cache->clone();
    events::cache_found op(base_foraging_controller::server(), clone.get());
    map()->accept(op);
    clone.reset();
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
  if (base_sensors()->tick() > 2) {
    return base_sensors()->robot_heading().Length();
  }
  return 0;
} /* timestep_distance() */

/*******************************************************************************
 * Stateless Diagnostics
 ******************************************************************************/
bool foraging_controller::is_exploring_for_block(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_exploring_for_block();
  }
  return false;
} /* is_exploring_for_block() */

bool foraging_controller::is_avoiding_collision(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_avoiding_collision();
  }
  return false;
} /* is_avoiding_collision() */

bool foraging_controller::is_transporting_to_nest(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_transporting_to_nest();
  }
  return false;
} /* is_transporting_to_nest() */

/*******************************************************************************
 * Stateful Diagnostics
 ******************************************************************************/
bool foraging_controller::is_acquiring_block(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_acquiring_block();
  }
  return false;
} /* is_exploring() */

bool foraging_controller::is_vectoring_to_block(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_vectoring_to_block();
  }
  return false;
} /* is_vectoring_to_block() */

/*******************************************************************************
 * Depth1 Diagnostics
 ******************************************************************************/
bool foraging_controller::is_exploring_for_cache(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_exploring_for_cache();
  }
  return false;
} /* is_exploring_for_cache() */

bool foraging_controller::is_vectoring_to_cache(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_vectoring_to_cache();
  }
  return false;
} /* is_vectoring_to_cache() */

bool foraging_controller::is_acquiring_cache(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_acquiring_cache();
  }
  return false;
} /* is_acquring_to_cache() */

bool foraging_controller::is_transporting_to_cache(void) const {
  if (nullptr != current_task()) {
    return current_task()->is_transporting_to_cache();
  }
  return false;
} /* is_transporting_to_cache() */

std::string foraging_controller::task_name(void) const {
  if (nullptr != current_task()) {
    return current_task()->task_name();
  }
  return "";
} /* task_name() */

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
using depth1_foraging_controller = foraging_controller;
REGISTER_CONTROLLER(depth1_foraging_controller, "depth1_foraging_controller"); // NOLINT

NS_END(depth1, controller, fordyca);
