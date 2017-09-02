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
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/controller/unpartitioned_task_fsm.hpp"
#include "fordyca/controller/block_target_selector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
unpartitioned_task_fsm::unpartitioned_task_fsm(
    const struct foraging_fsm_params* params,
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<sensor_manager>& sensors,
    const std::shared_ptr<actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    fsm::hfsm(server),
    start(),
    return_to_nest(),
    leaving_nest(),
    entry_return_to_nest(),
    entry_leaving_nest(),
    exit_leaving_nest(),
    locate_block(),
    explore(),
    new_direction(),
    collision_avoidance(),
    entry_new_direction(),
    entry_explore(),
    entry_collision_avoidance(),
    exit_locate_block(),
    m_current_state(ST_START),
    m_next_state(ST_START),
    m_initial_state(ST_START),
    m_previous_state(ST_START),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_state(),
    mc_params(params),
    m_sensors(sensors),
    m_actuators(actuators),
    m_map(map),
    m_vector_fsm(params->times.frequent_collision_thresh,
                 server, sensors, actuators) {
  insmod("unpartitioned_task_fsm");
  server_handle()->mod_loglvl(er_id(), rcppsw::common::er_lvl::DIAG);
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::NOM);
    }

/*******************************************************************************
 * Events
 ******************************************************************************/
void unpartitioned_task_fsm::event_block_found(void) {
  HFSM_DEFINE_TRANSITION_MAP(kTRANSITIONS) {
        ST_RETURN_TO_NEST,           /* start (robot might be on block initially) */
        ST_RETURN_TO_NEST,           /* explore */
        ST_RETURN_TO_NEST,           /* new direction */
        fsm::event_signal::IGNORED,  /* return to nest */
        fsm::event_signal::FATAL,    /* leaving nest */
        ST_COLLISION_AVOIDANCE,      /* collision avoidance */
        ST_RETURN_TO_NEST,           /* locate block */
        };
  HFSM_VERIFY_TRANSITION_MAP(kTRANSITIONS);
  external_event(kTRANSITIONS[current_state()],
                 rcppsw::make_unique<fsm::event_data>(foraging_signal::BLOCK_FOUND,
                                                      fsm::event_type::NORMAL));
}

/*******************************************************************************
 * Exploration FSM
 ******************************************************************************/
HFSM_STATE_DEFINE(unpartitioned_task_fsm, explore, fsm::event_data) {
  ER_DIAG("Executing ST_EXPLORE");

  ++m_state.time_exploring_unsuccessfully;
  argos::CVector2 a;
  /*
   * Check for nearby obstacles, and if so go int obstacle avoidance. Time spent
   * in collision avoidance still counts towards the direction change threshold.
   */
  if (m_sensors->calc_diffusion_vector(NULL)) {
    internal_event(ST_COLLISION_AVOIDANCE);
  } else if (m_state.time_exploring_unsuccessfully >
             mc_params->times.unsuccessful_explore_dir_change) {
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
HFSM_STATE_DEFINE(unpartitioned_task_fsm, collision_avoidance, fsm::event_data) {
  argos::CVector2 vector;
  ER_DIAG("Executing ST_COLLIISION_AVOIDANCE");

  if (m_sensors->calc_diffusion_vector(&vector)) {
    m_actuators->set_heading(vector);
  } else {
    internal_event(previous_state());
  }
  return fsm::event_signal::HANDLED;
}

HFSM_STATE_DEFINE(unpartitioned_task_fsm, new_direction, new_direction_data) {
  ER_DIAG("Executing ST_NEW_DIRECTION");
  static argos::CRadians new_dir;
  static int count = 0;
  argos::CRadians current_dir = m_sensors->calc_vector_to_light().Angle();

  /*
   * The new direction is only passed the first time this state is entered, so
   * save it.
   */
  if (data) {
    count = 0;
    new_dir = data->dir;
  }
  m_actuators->set_heading(argos::CVector2(m_actuators->max_wheel_speed() * 0.25,
                                           new_dir), true);

  /* We have changed direction and started a new exploration */
  if (std::fabs((current_dir - new_dir).GetValue()) < 0.1) {
    m_state.time_exploring_unsuccessfully = 0;
    internal_event(ST_EXPLORE);
  }
  ++count;
  return fsm::event_signal::HANDLED;
}
HFSM_ENTRY_DEFINE(unpartitioned_task_fsm, entry_explore, fsm::no_event_data) {
  ER_DIAG("Entering ST_EXPLORE");
  m_actuators->leds_set_color(argos::CColor::MAGENTA);
}
HFSM_ENTRY_DEFINE(unpartitioned_task_fsm, entry_new_direction, fsm::no_event_data) {
  ER_DIAG("Entering ST_NEW_DIRECTION");
  m_actuators->leds_set_color(argos::CColor::CYAN);
}
HFSM_ENTRY_DEFINE(unpartitioned_task_fsm, entry_collision_avoidance, fsm::no_event_data) {
  ER_DIAG("Entering ST_COLLISION_AVOIDANCE");
  m_actuators->leds_set_color(argos::CColor::RED);
}

/*******************************************************************************
 * Locate Block FSM
 ******************************************************************************/
HFSM_STATE_DEFINE(unpartitioned_task_fsm, locate_block, fsm::event_data) {
  ER_NOM("Executing ST_LOCATE_BLOCK");

  /*
   * We are executing this state as part of the normal worker process.
   */
  if (fsm::event_type::NORMAL == data->type()) {
    /* We found a known block */
    if (acquire_block()) {
      internal_event(ST_RETURN_TO_NEST);
    }
  } else if (fsm::event_type::CHILD == data->type()) {
    /* We have found a block through the exploration sub-fsm */
    if (foraging_signal::BLOCK_FOUND == data->signal()) {
      internal_event(ST_RETURN_TO_NEST);
    }
  }
  return fsm::event_signal::HANDLED;
}

HFSM_EXIT_DEFINE(unpartitioned_task_fsm, exit_locate_block) {
  m_vector_fsm.init();
}
/*******************************************************************************
 * Non-Hierarchical States
 ******************************************************************************/
HFSM_STATE_DEFINE(unpartitioned_task_fsm, leaving_nest, fsm::no_event_data) {
  ER_DIAG("Executing ST_LEAVING_NEST");

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
                   rcppsw::make_unique<fsm::event_data>(foraging_signal::BLOCK_FOUND,
                                                        fsm::event_type::NORMAL));
  }
  return fsm::event_signal::HANDLED;
}
HFSM_STATE_DEFINE(unpartitioned_task_fsm, start, fsm::no_event_data) {
  internal_event(ST_LOCATE_BLOCK, rcppsw::make_unique<fsm::event_data>(fsm::event_signal::IGNORED,
                                                                       fsm::event_type::NORMAL));
  return fsm::event_signal::HANDLED;
}
HFSM_STATE_DEFINE(unpartitioned_task_fsm, return_to_nest, fsm::no_event_data) {
  ER_DIAG("Executing ST_RETURN_TO_NEST");
  argos::CVector2 vector;

  /*
   * We have arrived at the nest and it's time to head back out again. The
   * loop functions need to call the drop_block() function, as they have to
   * redistribute it (the FSM has no idea how to do that).
   */
  if (m_sensors->in_nest()) {
    internal_event(ST_LEAVING_NEST);
  }

  /* ignore all obstacles for now... */
  m_actuators->set_heading(m_actuators->max_wheel_speed() *
                           m_sensors->calc_vector_to_light());
  return fsm::event_signal::HANDLED;
}
HFSM_ENTRY_DEFINE(unpartitioned_task_fsm, entry_leaving_nest, fsm::no_event_data) {
  ER_DIAG("Entering ST_LEAVING_NEST");
  m_actuators->leds_set_color(argos::CColor::WHITE);
}
HFSM_ENTRY_DEFINE(unpartitioned_task_fsm, entry_return_to_nest, fsm::no_event_data) {
  ER_DIAG("Entering ST_RETURN_TO_NEST");
  m_actuators->leds_set_color(argos::CColor::GREEN);
}
HFSM_EXIT_DEFINE(unpartitioned_task_fsm, exit_leaving_nest) {
  ER_DIAG("Exiting ST_LEAVING_NEST");
  m_state.time_exploring_unsuccessfully = 0;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void unpartitioned_task_fsm::init(void) {
  m_state.time_exploring_unsuccessfully = 0;
  m_actuators->reset();
  base_fsm::init();
  m_vector_fsm.init();
} /* init() */

argos::CVector2 unpartitioned_task_fsm::randomize_vector_angle(argos::CVector2 vector) {
  argos::CRange<argos::CRadians> range(argos::CRadians(0.0),
                                       argos::CRadians(1.0));
  vector.Rotate(m_rng->Uniform(range));
  return vector;
} /* randomize_vector_angle() */

void unpartitioned_task_fsm::update_state(uint8_t new_state) {
  if (new_state != m_current_state) {
    m_previous_state = m_current_state;
  }
  m_current_state = new_state;
} /* update_state() */

void unpartitioned_task_fsm::acquire_known_block(
    std::list<std::pair<const representation::block*, double>> blocks) {
  block_target_selector selector(mc_params->nest_center);
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


NS_END(controller, fordyca);
