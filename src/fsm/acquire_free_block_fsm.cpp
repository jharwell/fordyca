/**
 * @file acquire_free_block_fsm.cpp
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
#include "fordyca/fsm/acquire_free_block_fsm.hpp"
#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/controller/sensor_manager.hpp"
#include "fordyca/controller/block_selector.hpp"
#include "fordyca/controller/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_free_block_fsm::acquire_free_block_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::common::er_server>& server,
    const std::shared_ptr<controller::sensor_manager>& sensors,
    const std::shared_ptr<controller::actuator_manager>& actuators,
    const std::shared_ptr<const representation::perceived_arena_map>& map) :
    base_foraging_fsm(server, sensors, actuators, ST_MAX_STATES),
    HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
    HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
    exit_acquire_block(),
    mc_nest_center(params->nest_center),
    m_rng(argos::CRandom::CreateRNG("argos")),
    m_map(map),
    m_server(server),
    m_vector_fsm(params->times.frequent_collision_thresh,
                 server, sensors, actuators),
    m_explore_fsm(params->times.unsuccessful_explore_dir_change,
                  server, sensors, actuators) {
  m_explore_fsm.change_parent(explore_fsm::ST_EXPLORE, &acquire_block);
}

HFSM_STATE_DEFINE(acquire_free_block_fsm, acquire_block, state_machine::event_data) {
  if (ST_ACQUIRE_BLOCK != last_state()) {
    ER_DIAG("Executing ST_ACQUIRE_BLOCK");
  }
  ER_ASSERT(data, "FATAL: No event data passed to ST_ACQUIRE_BLOCK");

  /*
   * We are executing this state as part of the normal block search process.
   */
  if (state_machine::event_type::NORMAL == data->type()) {
      /* We acquired a block */
      if (acquire_free_block()) {
        internal_event(ST_FINISHED);
      }
  } else if (state_machine::event_type::CHILD == data->type()) {
    /*
     * We have found a block through the exploration sub-fsm; vector to it and
     * pick it up
     */
    if (controller::foraging_signal::BLOCK_LOCATED == data->signal()) {
      ER_ASSERT(m_map->blocks().size(),
                "FATAL: Block 'located' but empty block list");

      /* We acquired a block */
      if (acquire_free_block()) {
        internal_event(ST_FINISHED);
      }
    }
  }
  return state_machine::event_signal::HANDLED;
}

HFSM_EXIT_DEFINE(acquire_free_block_fsm, exit_acquire_block) {
  m_vector_fsm.task_reset();
  m_explore_fsm.init();
}
FSM_STATE_DEFINE(acquire_free_block_fsm, finished, state_machine::no_event_data) {
  if (ST_FINISHED != last_state()) {
    ER_DIAG("Executing ST_FINISHED");
  }
  return state_machine::event_signal::HANDLED;
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void acquire_free_block_fsm::init(void) {
  base_foraging_fsm::init();
  m_vector_fsm.task_reset();
  m_explore_fsm.init();
} /* init() */

void acquire_free_block_fsm::acquire_known_block(
    std::list<std::pair<const representation::block*, double>> blocks) {
  controller::block_selector selector(m_server, mc_nest_center);
  auto best = selector.calc_best(blocks,
                                 base_foraging_fsm::sensors()->robot_loc());
  ER_NOM("Vector towards best block: %d@(%zu, %zu)=%f",
         best.first->id(),
         best.first->discrete_loc().first,
         best.first->discrete_loc().second,
         best.second);
  vector_argument v(best.first->real_loc());
  m_vector_fsm.task_start(&v);
} /* acquire_known_block() */

bool acquire_free_block_fsm::acquire_free_block(void) {
  /* currently on our way to a known block */
  if (m_vector_fsm.in_progress()) {
    m_vector_fsm.task_execute();
     return false;
  } else if (m_vector_fsm.task_finished()) {
    if (base_foraging_fsm::sensors()->block_detected()) {
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
    m_explore_fsm.run();
  }
  return false;
} /* acquire_free_block() */

void acquire_free_block_fsm::task_execute(void) {
  inject_event(state_machine::event_signal::IGNORED,
               state_machine::event_type::NORMAL);
} /* task_execute() */


NS_END(controller, fordyca);
