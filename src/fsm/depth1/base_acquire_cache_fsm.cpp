/**
 * @file base_acquire_cache_fsm.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/fsm/depth1/base_acquire_cache_fsm.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/color.h>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/existing_cache_selector.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm, depth1);
namespace state_machine = rcppsw::patterns::state_machine;
namespace depth1 = controller::depth1;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_acquire_cache_fsm::base_acquire_cache_fsm(
    const struct params::fsm_params* params,
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<controller::saa_subsystem>& saa,
    std::shared_ptr<const representation::perceived_arena_map> map)
    : base_foraging_fsm(server, saa, ST_MAX_STATES),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(acquire_cache, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      exit_acquire_cache(),
      mc_nest_center(params->nest_center),
      m_rng(argos::CRandom::CreateRNG("argos")),
      mc_map(std::move(map)),
      m_server(server),
      m_vector_fsm(server, saa),
      m_explore_fsm(server, saa),
      mc_state_map{HFSM_STATE_MAP_ENTRY_EX(&start),
                   HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_cache,
                                               nullptr,
                                               nullptr,
                                               &exit_acquire_cache),
                   HFSM_STATE_MAP_ENTRY_EX(&finished)} {
  m_explore_fsm.change_parent(explore_for_cache_fsm::ST_EXPLORE, &acquire_cache);
}

HFSM_STATE_DEFINE_ND(base_acquire_cache_fsm, start) {
  ER_DIAG("Executing ST_START");
  internal_event(ST_ACQUIRE_CACHE);
  return controller::foraging_signal::HANDLED;
}

HFSM_STATE_DEFINE_ND(base_acquire_cache_fsm, acquire_cache) {
  if (ST_ACQUIRE_CACHE != last_state()) {
    ER_DIAG("Executing ST_ACQUIRE_CACHE");
  }

  if (acquire_any_cache()) {
    internal_event(ST_FINISHED);
  }
  return state_machine::event_signal::HANDLED;
}

HFSM_EXIT_DEFINE(base_acquire_cache_fsm, exit_acquire_cache) {
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
}
HFSM_STATE_DEFINE_ND(base_acquire_cache_fsm, finished) {
  if (ST_FINISHED != last_state()) {
    ER_DIAG("Executing ST_FINISHED");
  }

  return state_machine::event_signal::HANDLED;
}

/*******************************************************************************
 * Metrics
 ******************************************************************************/
__pure bool base_acquire_cache_fsm::is_avoiding_collision(void) const {
  return m_explore_fsm.is_avoiding_collision() ||
         m_vector_fsm.is_avoiding_collision();
} /* is_avoiding_collision() */

bool base_acquire_cache_fsm::cache_acquired(void) const {
  return current_state() == ST_FINISHED;
} /* cache_acquired() */

bool base_acquire_cache_fsm::is_exploring_for_cache(void) const {
  return (current_state() == ST_ACQUIRE_CACHE && m_explore_fsm.task_running());
} /* is_exploring_for_cache() */

bool base_acquire_cache_fsm::is_vectoring_to_cache(void) const {
  return current_state() == ST_ACQUIRE_CACHE && m_vector_fsm.task_running();
} /* is_vectoring_to_cache() */

bool base_acquire_cache_fsm::is_acquiring_cache(void) const {
  return is_vectoring_to_cache() || is_exploring_for_cache();
} /* is_acquring_cache() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void base_acquire_cache_fsm::init(void) {
  base_foraging_fsm::init();
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
} /* init() */

bool base_acquire_cache_fsm::acquire_known_cache(
    std::list<representation::perceived_cache> caches) {
  /*
   * If we don't know of any caches and we are not current vectoring towards
   * one, then there is no way we can acquire a known cache, so bail out.
   */
  if (caches.empty() && !m_vector_fsm.task_running()) {
    return false;
  }

  if (!caches.empty() && !m_vector_fsm.task_running()) {
    /*
     * If we get here, we must know of some caches, but not be currently
     * vectoring toward any of them.
     */
    if (!m_vector_fsm.task_running()) {
      controller::depth1::existing_cache_selector selector(server_ref(),
                                                           mc_nest_center);
      tasks::vector_argument v(vector_fsm::kCACHE_ARRIVAL_TOL,
                               select_cache_for_acquisition());
      m_explore_fsm.task_reset();
      m_vector_fsm.task_reset();
      m_vector_fsm.task_start(&v);
    }
  }

  /* we are vectoring */
  if (!m_vector_fsm.task_finished()) {
    m_vector_fsm.task_execute();
  }

  if (m_vector_fsm.task_finished()) {
    m_vector_fsm.task_reset();

    const auto& sensors =
        std::static_pointer_cast<depth1::sensing_subsystem>(base_sensors());
    if (sensors->cache_detected()) {
      return true;
    }
    ER_WARN("WARNING: Robot arrived at goal, but no cache was detected.");
    return false;
  }
  return false;
} /* acquire_known_cache() */

bool base_acquire_cache_fsm::acquire_unknown_cache(void) {
  if (!m_explore_fsm.task_running()) {
    m_explore_fsm.task_reset();
    m_explore_fsm.task_start(nullptr);
  }
  m_explore_fsm.task_execute();
  if (m_explore_fsm.task_finished()) {
    const auto& sensors =
        std::static_pointer_cast<depth1::sensing_subsystem>(base_sensors());
    ER_ASSERT(sensors->cache_detected(),
              "FATAL: No cache detected after successful exploration");
    return true;
  }
  return false;
} /* acquire_unknown_cache() */

bool base_acquire_cache_fsm::acquire_any_cache(void) {
  /*
   * If we know of ANY caches in the arena, go to the location of the best one
   * and pick it up. Otherwise, explore until you find one. If during
   * exploration we find one through our LOS, then stop exploring and go vector
   * to it.
   */
  if (!acquire_known_cache(mc_map->perceived_caches())) {
    if (m_vector_fsm.task_running()) {
      return false;
    }

    /* try again--someone beat us to our chosen cache */
    return acquire_unknown_cache();
  }
  return true;
} /* acquire_any_cache() */

void base_acquire_cache_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN,
               state_machine::event_type::NORMAL);
} /* task_execute() */

NS_END(depth1, controller, fordyca);
