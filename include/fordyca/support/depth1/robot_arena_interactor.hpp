/**
 * @file robot_arena_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_ROBOT_ARENA_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_ROBOT_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "fordyca/support/depth0/robot_arena_interactor.hpp"

#include "fordyca/support/task_abort_interactor.hpp"
#include "fordyca/support/cached_block_pickup_interactor.hpp"
#include "fordyca/support/existing_cache_block_drop_interactor.hpp"
#include "fordyca/support/cache_op_penalty_handler.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class robot_arena_interactor
 * @ingroup support depth1
 *
 * @brief Handles a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 * - Picking up from/dropping a block in a cache.
 * - Subjecting robots using caches to a penalty on cache drop/pickup.
 * - Picking up a free block.
 * - Dropping a carried block in the nest.
 * - Free block drop due to task abort.
 */
template <typename T>
class robot_arena_interactor : public depth0::robot_arena_interactor<T>,
                         public er::client<robot_arena_interactor<T>> {
 public:
  robot_arena_interactor(ds::arena_map* const map_in,
                         depth0::depth0_metrics_aggregator *const metrics_agg,
                         argos::CFloorEntity* const floor_in,
                         const ct::waveform_params* const block_manip_penalty,
                         const ct::waveform_params* const cache_usage_penalty)
      : depth0::robot_arena_interactor<T>(map_in,
                                          metrics_agg,
                                          floor_in,
                                          block_manip_penalty),
    ER_CLIENT_INIT("fordyca.support.depth1.robot_arena_interactor"),
    m_cache_penalty_handler(map_in, cache_usage_penalty, "Cache"),
    m_task_abort_interactor(map_in, floor_in),
    m_cached_pickup_interactor(map_in, floor_in, &m_cache_penalty_handler),
    m_existing_cache_drop_interactor(map_in,
                                     floor_in,
                                     &m_cache_penalty_handler) {}

  robot_arena_interactor& operator=(
      const robot_arena_interactor& other) = delete;
  robot_arena_interactor(const robot_arena_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    std::list<temporal_penalty_handler<T>*> penalty_handlers =  {
      nest_drop_interactor().penalty_handler(),
      free_pickup_interactor().penalty_handler(),
      &m_cache_penalty_handler
    };
    if (m_task_abort_interactor(controller, penalty_handlers)) {
      return;
    }

    if (controller.is_carrying_block()) {
      nest_drop_interactor()(controller, timestep);
      m_existing_cache_drop_interactor(controller, timestep);
    } else { /* The foot-bot has no block item */
      free_pickup_interactor()(controller, timestep);
      m_cached_pickup_interactor(controller, timestep);
    }
  }

  /**
   * @brief Given the current timestep, get the value of the cache usage penalty
   * waveform that would apply if a robot were to start serving a penalty
   * now. Cache usage penalty is the same no matter what the robot is doing, so
   * it can safely be used for all cases (pickup, drop, etc.).
   */
  double cache_usage_penalty(double t) const {
    return m_cache_penalty_handler.timestep_penalty(t);
  }

 protected:
  using depth0::robot_arena_interactor<T>::nest_drop_interactor;
  using depth0::robot_arena_interactor<T>::free_pickup_interactor;

  const cached_block_pickup_interactor<T>& cached_pickup_interactor(void) const {
    return m_cached_pickup_interactor;
  }
  cached_block_pickup_interactor<T>& cached_pickup_interactor(void) {
    return m_cached_pickup_interactor;
  }
  const existing_cache_block_drop_interactor<T>& existing_cache_drop_interactor(void) const {
    return m_existing_cache_drop_interactor;
  }
  existing_cache_block_drop_interactor<T>& existing_cache_drop_interactor(void) {
    return m_existing_cache_drop_interactor;
  }
  const support::task_abort_interactor<T>& task_abort_interactor(void) const {
    return m_task_abort_interactor;
  }
  support::task_abort_interactor<T>& task_abort_interactor(void) {
    return m_task_abort_interactor;
  }


  const cache_op_penalty_handler<T>& cache_penalty_handler(void) const {
    return m_cache_penalty_handler;
  }
  cache_op_penalty_handler<T>& cache_penalty_handler(void) {
    return m_cache_penalty_handler;
  }

 private:
  // clang-format off
  cache_op_penalty_handler<T>             m_cache_penalty_handler;
  support::task_abort_interactor<T>       m_task_abort_interactor;
  cached_block_pickup_interactor<T>       m_cached_pickup_interactor;
  existing_cache_block_drop_interactor<T> m_existing_cache_drop_interactor;
  // clang-format on
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_ROBOT_ARENA_INTERACTOR_HPP_ */
