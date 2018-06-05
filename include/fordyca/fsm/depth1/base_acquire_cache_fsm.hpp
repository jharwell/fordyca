/**
 * @file base_acquire_cache_fsm.hpp
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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_BASE_ACQUIRE_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_BASE_ACQUIRE_CACHE_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/rng.h>

#include "rcppsw/task_allocation/taskable.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/vector_fsm.hpp"
#include "fordyca/fsm/depth1/explore_for_cache_fsm.hpp"
#include "fordyca/representation/perceived_cache.hpp"

#include "fordyca/metrics/fsm/cache_acquisition_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct fsm_params; }
namespace representation { class perceived_arena_map; class cache; }

NS_START(fsm, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_acquire_cache_fsm
 * @ingroup fsm depth1
 *
 * @brief The base FSM for an acquiring from a cache in the arena.
 *
 * Each robot executing this FSM will look for a cache (either a known cache or
 * via random exploration). Once a cache has been acquired, it signals that it
 * has completed its task.
 */
class base_acquire_cache_fsm : public base_foraging_fsm,
                               public metrics::fsm::cache_acquisition_metrics,
                               public rcppsw::task_allocation::taskable {
 public:
  base_acquire_cache_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::saa_subsystem>& saa,
      std::shared_ptr<const representation::perceived_arena_map> map);

  base_acquire_cache_fsm(const base_acquire_cache_fsm& fsm) = delete;
  base_acquire_cache_fsm& operator=(const base_acquire_cache_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(__unused const rcppsw::task_allocation::taskable_argument* const arg) override {}
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override { return ST_ACQUIRE_CACHE == current_state(); }
  void task_reset(void) override { init(); }

  /* base FSM metrics */
  bool is_avoiding_collision(void) const override;

  /* cache acquisition metrics */
  bool is_exploring_for_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool cache_acquired(void) const override;

  /**
   * @brief Reset the FSM
   */
  void init(void) override;

 protected:
  enum fsm_states {
    ST_START,
    ST_ACQUIRE_CACHE, /* superstate for finding a cache */
    ST_FINISHED,
    ST_MAX_STATES
  };

  /**
   * @brief Get the cache location corresponding to the "best" cache (by some
   * measure), for use in vectoring.
   *
   * @return The location of the "best" cache to acquire
   */
  virtual argos::CVector2 select_cache_for_acquisition(void) = 0;

  argos::CVector2 nest_center(void) const { return mc_nest_center; }
  std::shared_ptr<const representation::perceived_arena_map> map(void) const {
    return mc_map;
  }

 private:
  /**
   * @brief Acquire a known cache or discover one via random exploration.
   *
   * @return \c TRUE if a cache has been acquired, \c FALSE otherwise.
   */
  bool acquire_any_cache(void);

  /**
   * @brief Acquire an unknown cache via exploration.
   *
   * @return \c TRUE if a cache has been acquired \c FALSE otherwise.
   */
  bool acquire_unknown_cache(void);

  /**
   * @brief Acquire a known cache. If the robot's knowledge of the chosen
   * cache's existence expires during the pursuit of said cache, that is
   * ignored.
   *
   * @param caches The list of perceived caches. This CANNOT be a reference, as
   * the robot's list of perceived caches can change during the course of
   * acquiring a cache, and refering to specific positions within the vector
   * that the robot maintains leads to...interesting behavior.
   */
  bool acquire_known_cache(
      std::list<representation::perceived_cache> caches);

  HFSM_STATE_DECLARE_ND(base_acquire_cache_fsm, start);
  HFSM_STATE_DECLARE_ND(base_acquire_cache_fsm, acquire_cache);
  HFSM_STATE_DECLARE_ND(base_acquire_cache_fsm, finished);

  HFSM_EXIT_DECLARE(base_acquire_cache_fsm, exit_acquire_cache);

  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  // clang-format off
  const argos::CVector2                                      mc_nest_center;
  argos::CRandom::CRNG*                                      m_rng;
  std::shared_ptr<const representation::perceived_arena_map> mc_map;
  std::shared_ptr<rcppsw::er::server>                        m_server;
  vector_fsm                                                 m_vector_fsm;
  explore_for_cache_fsm                                      m_explore_fsm;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_BASE_ACQUIRE_CACHE_FSM_HPP_ */
