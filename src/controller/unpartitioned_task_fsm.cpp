/**
 * @file unpartitioned_task_fsm.cpp
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
#include "fordyca/controller/unpartitioned_task_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/block_target_selector.hpp"
#include "fordyca/params/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
unpartitioned_task_fsm::unpartitioned_task_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<sensor_manager>& sensors,
    const std::shared_ptr<actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    random_foraging_fsm(params, server, sensors, actuators),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(return_to_nest, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
    entry_return_to_nest(),
    entry_leaving_nest(),
    exit_leaving_nest(),
    HFSM_CONSTRUCT_STATE(locate_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(explore, &locate_block),
    HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
    entry_new_direction(),
    entry_explore(),
    entry_collision_avoidance(),
    exit_locate_block(),
    mc_unsuccessful_explore_dir_change(params->times.unsuccessful_explore_dir_change),
    mc_nest_center(params->nest_center),
    m_current_state(ST_START),
    m_next_state(ST_START),
    m_initial_state(ST_START),
    m_previous_state(ST_START),
    m_last_state(ST_START),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_state(),
    m_sensors(sensors),
    m_actuators(actuators),
    m_map(map),
    m_server(server),
    m_vector_fsm(params->times.frequent_collision_thresh,
                 server, sensors, actuators) {
    }

/*******************************************************************************
 * Exploration FSM
 ******************************************************************************/
HFSM_STATE_DEFINE(unpartitioned_task_fsm, explore, fsm::event_data) {
  if (ST_EXPLORE != last_state()) {
    ER_DIAG("Executing ST_EXPLORE");
  }
  if (data) {
    ER_ASSERT(fsm::event_type::NORMAL == data->type(),
              "FATAL: ST_EXPLORE cannot handle child events");
    ER_ASSERT(foraging_signal::BLOCK_ACQUIRED != data->signal(),
              "FATAL: ST_EXPLORE should never acquire blocks...");
    if (foraging_signal::BLOCK_LOCATED == data->signal()) {
      return fsm::event_signal::UNHANDLED;
    }
  }

  explore_time_inc();

  /*
   * Check for nearby obstacles, and if so go into obstacle avoidance. Time spent
   * in collision avoidance still counts towards the direction change threshold.
   */
  if (m_sensors->calc_diffusion_vector(NULL)) {
    internal_event(ST_COLLISION_AVOIDANCE);
  } else if (explore_time() > mc_unsuccessful_explore_dir_change) {
    argos::CRange<argos::CRadians> range(argos::CRadians(0.50),
                                         argos::CRadians(1.0));
    argos::CVector2 new_dir = randomize_vector_angle(argos::CVector2::X);
    internal_event(ST_NEW_DIRECTION,
                   rcppsw::make_unique<new_direction_data>(new_dir.Angle()));
  }
  /*
   * No obstacles nearby and have not hit direction changing threshold--use the
   * diffusion vector only to set speeds.
   */
  argos::CVector2 vector;
  m_sensors->calc_diffusion_vector(&vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() * vector);
  return fsm::event_signal::HANDLED;
}

HFSM_STATE_DEFINE(unpartitioned_task_fsm, leaving_nest, fsm::no_event_data) {
  if (ST_LEAVING_NEST != last_state()) {
    ER_DIAG("Executing ST_LEAVING_NEST");
  }
  /*
   * The vector returned by calc_vector_to_light() points to the light. Thus,
   * the minus sign is because we want to go away from the light.
   */
  argos::CVector2 diff_vector;
  argos::CRadians current_heading = m_sensors->calc_vector_to_light().Angle();
  m_sensors->calc_diffusion_vector(&diff_vector);
  m_actuators->set_heading(m_actuators->max_wheel_speed() * diff_vector -
                           argos::CVector2(m_actuators->max_wheel_speed() * 0.25f,
                                           current_heading));
  if (!m_sensors->in_nest()) {
    internal_event(ST_LOCATE_BLOCK,
                   rcppsw::make_unique<fsm::event_data>(fsm::event_signal::IGNORED,
                                                        fsm::event_type::NORMAL));
  }
  return fsm::event_signal::HANDLED;
}

/*******************************************************************************
 * Locate Block FSM
 ******************************************************************************/
HFSM_STATE_DEFINE(unpartitioned_task_fsm, locate_block, fsm::event_data) {
  if (ST_LOCATE_BLOCK != last_state()) {
    ER_DIAG("Executing ST_LOCATE_BLOCK");
  }
  ER_ASSERT(data, "FATAL: No event data passed to ST_LOCATE_BLOCK");

  /*
   * We are executing this state as part of the normal worker process.
   */
  if (fsm::event_type::NORMAL == data->type()) {
      /* We acquired a known block */
      if (acquire_block()) {
        internal_event(ST_RETURN_TO_NEST);
      }
  } else if (fsm::event_type::CHILD == data->type()) {
    /*
     * This is the first time we have been called via the exploration sub-fsm;
     * we will take over execution.
     */
    if (!m_vector_fsm.in_progress()) {
      update_state(ST_LOCATE_BLOCK);
    }

    /*
     * We have found a block through the exploration sub-fsm; vector to it and
     * pick it up
     */
    if (foraging_signal::BLOCK_LOCATED == data->signal()) {
      ER_ASSERT(m_map->blocks().size(),
                "FATAL: Block 'located' but empty block list");

      /* We acquired a known block */
      if (acquire_block()) {
        internal_event(ST_RETURN_TO_NEST);
      }
    }
  }
  return fsm::event_signal::HANDLED;
}

HFSM_EXIT_DEFINE(unpartitioned_task_fsm, exit_locate_block) {
  m_vector_fsm.init();
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void unpartitioned_task_fsm::init(void) {
  explore_time_reset();
  m_actuators->reset();
  base_fsm::init();
  m_vector_fsm.init();
} /* init() */

void unpartitioned_task_fsm::update_state(uint8_t new_state) {
  if (new_state != m_current_state) {
    m_previous_state = m_current_state;
  }
  m_last_state = m_current_state;
  m_current_state = new_state;
} /* update_state() */

void unpartitioned_task_fsm::acquire_known_block(
    std::list<std::pair<const representation::block*, double>> blocks) {
  block_target_selector selector(m_server, mc_nest_center);
  auto best = selector.calc_best(blocks, m_sensors->robot_loc());
  ER_NOM("Vector towards best block: %d@(%zu, %zu)=%f",
         best.first->id(),
         best.first->discrete_loc().first,
         best.first->discrete_loc().second,
         best.second);
  m_vector_fsm.event_start(best.first->real_loc());
} /* acquire_known_block() */

bool unpartitioned_task_fsm::acquire_block(void) {
  /* currently on our way to a known block */
  if (m_vector_fsm.in_progress()) {
    m_vector_fsm.run();
     return false;
  } else if (m_vector_fsm.arrived_at_goal()) {
    if (m_sensors->block_detected()) {
      return true;
    } else {
      ER_WARN("WARNING: Robot arrived at goal, but no block was detected.");
      m_vector_fsm.init();
     }
  }
  /* try again--someone beat us to our chosen block */

  /*
   * If we know of ANY blocks in the arena, go to the location of the best one
   * and pick it up. Otherwise, explore until you find one.
   */
  auto blocks = m_map->blocks();
  if (blocks.size()) {
    acquire_known_block(blocks);
  } else {
    internal_event(ST_EXPLORE);
  }
  return false;
} /* acquire_block() */

void unpartitioned_task_fsm::run(void) {
  inject_event(fsm::event_signal::IGNORED, fsm::event_type::NORMAL);
} /* run() */


NS_END(controller, fordyca);
