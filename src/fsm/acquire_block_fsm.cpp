/**
 * @file acquire_block_fsm.cpp
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
#include "fordyca/fsm/acquire_block_fsm.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>

#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/controller/depth0/block_selector.hpp"
#include "fordyca/controller/depth0/foraging_sensors.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_block_fsm::acquire_block_fsm(
    const struct params::fsm_params *params,
    const std::shared_ptr<rcppsw::er::server> &server,
    const std::shared_ptr<controller::depth0::foraging_sensors> &sensors,
    const std::shared_ptr<controller::actuator_manager> &actuators,
    std::shared_ptr<representation::perceived_arena_map> map)
    : base_foraging_fsm(
          params->times.unsuccessful_explore_dir_change,
          server,
          std::static_pointer_cast<controller::base_foraging_sensors>(sensors),
          actuators,
          ST_MAX_STATES),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_block, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      exit_acquire_block(),
      mc_nest_center(params->nest_center),
      m_best_block(nullptr),
      m_rng(argos::CRandom::CreateRNG("argos")),
      m_map(std::move(map)),
      m_server(server),
      m_sensors(sensors),
      m_vector_fsm(params->times.frequent_collision_thresh,
                   server,
                   sensors,
                   actuators),
      m_explore_fsm(params->times.unsuccessful_explore_dir_change,
                    server,
                    sensors,
                    actuators),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_block,
                                               NULL,
                                               NULL,
                                               &exit_acquire_block),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  client::insmod("acquire_block_fsm",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
}

HFSM_STATE_DEFINE_ND(acquire_block_fsm, start) {
  internal_event(ST_ACQUIRE_BLOCK);
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(acquire_block_fsm, acquire_block) {
  if (ST_ACQUIRE_BLOCK != last_state()) {
    ER_DIAG("Executing ST_ACQUIRE_BLOCK");
  }

  if (acquire_any_block()) {
    internal_event(ST_FINISHED);
  }
  return controller::foraging_signal::HANDLED;
}

HFSM_EXIT_DEFINE(acquire_block_fsm, exit_acquire_block) {
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
}
HFSM_STATE_DEFINE_ND(acquire_block_fsm, finished) {
  if (ST_FINISHED != last_state()) {
    ER_DIAG("Executing ST_FINISHED");
  }
  return controller::foraging_signal::HANDLED;
}

/*******************************************************************************
 * Base Diagnostics
 ******************************************************************************/
bool acquire_block_fsm::is_exploring_for_block(void) const {
  return (current_state() == ST_ACQUIRE_BLOCK && m_explore_fsm.task_running());
} /* is_exploring_for_block() */

__pure bool acquire_block_fsm::is_avoiding_collision(void) const {
  return m_explore_fsm.is_avoiding_collision() ||
         m_vector_fsm.is_avoiding_collision();
} /* is_avoiding_collision() */

/*******************************************************************************
 * Depth0 Diagnostics
 ******************************************************************************/
bool acquire_block_fsm::is_acquiring_block(void) const {
  return is_vectoring_to_block() || is_exploring_for_block();
} /* is_acquiring_block() */

bool acquire_block_fsm::is_vectoring_to_block(void) const {
  return current_state() == ST_ACQUIRE_BLOCK && m_vector_fsm.task_running();
} /* is_vectoring_to_block() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void acquire_block_fsm::init(void) {
  base_foraging_fsm::init();
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
} /* init() */

bool acquire_block_fsm::acquire_known_block(
    std::list<representation::const_perceived_block> blocks) {

  /*
   * If we don't know of any blocks and we are not current vectoring towards
   * one, then there is no way we can acquire a known block, so bail out.
   */
  if (blocks.empty() && !m_vector_fsm.task_running()) {
    return false;
  }

  if (!blocks.empty() && !m_vector_fsm.task_running()) {
    /*
     * If we get here, we must know of some blocks, but not be currently
     * vectoring toward any of them.
     */
    controller::depth0::block_selector selector(m_server, mc_nest_center);
    auto best = selector.calc_best(blocks, m_sensors->robot_loc());
    ER_NOM("Vector towards best block: %d@(%zu, %zu)=%f",
           best.first->id(),
           best.first->discrete_loc().first,
           best.first->discrete_loc().second,
           best.second);
    tasks::vector_argument v(vector_fsm::kBLOCK_ARRIVAL_TOL,
                             best.first->real_loc());
    m_best_block = const_cast<representation::block*>(best.first);
    m_explore_fsm.task_reset();
    m_vector_fsm.task_reset();
    m_vector_fsm.task_start(&v);
  }

  /* we are vectoring */
  if (!m_vector_fsm.task_finished()) {
    m_vector_fsm.task_execute();
  }

  if (m_vector_fsm.task_finished()) {
    m_vector_fsm.task_reset();
    if (m_sensors->block_detected()) {
      return true;
    }
    ER_WARN("WARNING: Robot arrived at goal, but no block was detected.");
    m_map->block_remove(*m_best_block);
    return false;
  }
  return false;
} /* acquire_known_block() */

bool acquire_block_fsm::acquire_any_block(void) {
  /*
   * If we know of ANY blocks in the arena, go to the location of the best one
   * and pick it up. Otherwise, explore until you find one. If during
   * exploration we find one through our LOS, then stop exploring and go vector
   * to it.
   */
  if (!acquire_known_block(m_map->perceived_blocks())) {
    if (m_vector_fsm.task_running()) {
      return false;
    }

    if (!m_explore_fsm.task_running()) {
      m_explore_fsm.task_reset();
      m_explore_fsm.task_start(nullptr);
    }
    m_explore_fsm.task_execute();
    if (m_explore_fsm.task_finished()) {
      ER_ASSERT(m_sensors->block_detected(),
                "FATAL: No block detected after successful exploration");
      return true;
    }
    return false;
  }
  return true;
} /* acquire_any_block() */

void acquire_block_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(controller, fordyca);
