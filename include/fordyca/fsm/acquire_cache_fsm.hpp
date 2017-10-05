/**
 * @file acquire_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_ACQUIRE_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_ACQUIRE_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <utility>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/fsm/explore_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct fsm_params;
} /* namespace params */

namespace controller {
class sensor_manager;
class actuator_manager;
} /* namespace controller */

namespace representation {
class perceived_arena_map;
class cache;
} /* namespace representation */

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 *@brief The FSM for an acquiring a block from a cache in the arena.
 *
 * Each robot executing this FSM will look for a cache (either a known cache or
 * via random exploration). Once a block has been acquired from an existing
 * cache has been acquired, it signals that it has completed its task.
 */
class acquire_cache_fsm : public base_foraging_fsm,
                                 public rcppsw::task_allocation::taskable {
 public:
  acquire_cache_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::common::er_server>& server,
      const std::shared_ptr<controller::sensor_manager>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<const representation::perceived_arena_map>& map);

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

  /**
   * @brief Get if the robot is currently searching for a cache within the arena.
   * (either vectoring towards a known block, or exploring for one).
   *
   * @return TRUE if the condition is met, FALSE otherwise.
   */
  bool is_searching_for_cache(void) const {
    return is_vectoring() || is_exploring();
  }

  bool is_exploring(void) const {
    return (current_state() == ST_ACQUIRE_CACHE && m_explore_fsm.is_searching()); }

  bool is_vectoring(void) const {
    return current_state() == ST_ACQUIRE_CACHE && m_vector_fsm.in_progress();
  }
  bool is_avoiding_collision(void) const {
    return m_explore_fsm.is_avoiding_collision();
  }
  void task_execute(void) override;
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_CACHE, /* superstate for finding a cache */
    ST_FINISHED,
    ST_MAX_STATES
  };

 private:
  /**
   * @brief Acquire a known cache or discover one via random exploration.
   *
   * @return TRUE if a block has been acquired, FALSE otherwise.
   */
  bool acquire_any_cache(void);

  /**
   * @brief Acquirea known cache.
   *
   * If the robot's knowledge of the chosen cache's existence expires during the
   * pursuit of said cache, that is ignored.
   */
  void acquire_known_cache(
      std::list<std::pair<const representation::cache*, double>> caches);

  /*
   * States for locate_block FSM. Note that the states for the vector_fsm
   * sub-fsm cannot be part of the locate_block hfsm, because that sub-fsm is
   * initiated from multiple states, and hfsm states can only have ONE parent
   * state.
   **/
  HFSM_STATE_DECLARE(acquire_cache_fsm, start,
                     state_machine::no_event_data);
  HFSM_STATE_DECLARE(acquire_cache_fsm, acquire_cache,
                     state_machine::event_data);
  HFSM_STATE_DECLARE(acquire_cache_fsm, finished,
                     state_machine::no_event_data);

  HFSM_EXIT_DECLARE(acquire_cache_fsm, exit_acquire_cache);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  HFSM_DEFINE_STATE_MAP(state_map_ex, kSTATE_MAP) {
    HFSM_STATE_MAP_ENTRY_EX(&start, hfsm::top_state()),
        HFSM_STATE_MAP_ENTRY_EX_ALL(&acquire_cache, hfsm::top_state(),
                                    NULL,
                                    NULL, &exit_acquire_cache),
        HFSM_STATE_MAP_ENTRY_EX(&finished, hfsm::top_state())
    };
  HFSM_VERIFY_STATE_MAP(state_map_ex, kSTATE_MAP);
  return &kSTATE_MAP[index];
  }

  acquire_cache_fsm(const acquire_cache_fsm& fsm) = delete;
  acquire_cache_fsm& operator=(const acquire_cache_fsm& fsm) = delete;

  const argos::CVector2 mc_nest_center;
  argos::CRandom::CRNG* m_rng;
  std::shared_ptr<const representation::perceived_arena_map> m_map;
  std::shared_ptr<rcppsw::common::er_server> m_server;
  vector_fsm m_vector_fsm;
  explore_fsm m_explore_fsm;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_ACQUIRE_CACHE_FSM_HPP_ */
