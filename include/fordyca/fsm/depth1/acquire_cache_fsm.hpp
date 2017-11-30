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

#ifndef INCLUDE_FORDYCA_FSM_DEPTH1_ACQUIRE_CACHE_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_DEPTH1_ACQUIRE_CACHE_FSM_HPP_

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
#include "fordyca/fsm/depth1/explore_for_cache_fsm.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/stateless_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/stateful_metrics.hpp"
#include "fordyca/metrics/collectible_metrics/robot_metrics/depth1_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct fsm_params; }
namespace representation { class perceived_arena_map; class cache; }
namespace controller {
namespace depth1 {class foraging_sensors; }
class actuator_manager;
}

NS_START(fsm, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class acquire_cache_fsm
 *
 * @brief The FSM for an acquiring a block from a cache in the arena.
 *
 * Each robot executing this FSM will look for a cache (either a known cache or
 * via random exploration). Once a block has been acquired from an existing
 * cache has been acquired, it signals that it has completed its task.
 */
class acquire_cache_fsm : public base_foraging_fsm,
                          public metrics::collectible_metrics::robot_metrics::stateless_metrics,
                          public metrics::collectible_metrics::robot_metrics::stateful_metrics,
                          public metrics::collectible_metrics::robot_metrics::depth1_metrics,
                          public rcppsw::task_allocation::taskable {
 public:
  acquire_cache_fsm(
      const struct params::fsm_params* params,
      const std::shared_ptr<rcppsw::er::server>& server,
      const std::shared_ptr<controller::depth1::foraging_sensors>& sensors,
      const std::shared_ptr<controller::actuator_manager>& actuators,
      const std::shared_ptr<const representation::perceived_arena_map>& map);

  /* taskable overrides */
  void task_execute(void) override;
  void task_start(__unused const rcppsw::task_allocation::taskable_argument* const arg) override {}
  bool task_finished(void) const override { return ST_FINISHED == current_state(); }
  bool task_running(void) const override { return ST_ACQUIRE_CACHE == current_state(); }
  void task_reset(void) override { init(); }

  /* base metrics */
  bool is_exploring_for_block(void) const override { return false; };
  bool is_avoiding_collision(void) const override;
  bool is_transporting_to_nest(void) const override { return false; }

  /* depth0 metrics */
  bool is_acquiring_block(void) const override { return false; };
  bool is_vectoring_to_block(void) const override { return false; };

  /* depth1 metrics */
  bool is_exploring_for_cache(void) const override;
  bool is_vectoring_to_cache(void) const override;
  bool is_acquiring_cache(void) const override;
  bool is_transporting_to_cache(void) const override { return false; };

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
  bool acquire_known_cache(
      std::list<std::pair<const representation::cache*, double>> caches);

  /*
   * States for locate_block FSM. Note that the states for the vector_fsm
   * sub-fsm cannot be part of the locate_block hfsm, because that sub-fsm is
   * initiated from multiple states, and hfsm states can only have ONE parent
   * state.
   **/
  HFSM_STATE_DECLARE_ND(acquire_cache_fsm, start);
  HFSM_STATE_DECLARE_ND(acquire_cache_fsm, acquire_cache);
  HFSM_STATE_DECLARE_ND(acquire_cache_fsm, finished);

  HFSM_EXIT_DECLARE(acquire_cache_fsm, exit_acquire_cache);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  acquire_cache_fsm(const acquire_cache_fsm& fsm) = delete;
  acquire_cache_fsm& operator=(const acquire_cache_fsm& fsm) = delete;

  const argos::CVector2                                      mc_nest_center;
  argos::CRandom::CRNG*                                      m_rng;
  std::shared_ptr<const representation::perceived_arena_map> m_map;
  std::shared_ptr<rcppsw::er::server>                        m_server;
  std::shared_ptr<controller::depth1::foraging_sensors>      m_sensors;
  vector_fsm                                                 m_vector_fsm;
  explore_for_cache_fsm                                      m_explore_fsm;
  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(depth1, fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_DEPTH1_ACQUIRE_CACHE_FSM_HPP_ */
