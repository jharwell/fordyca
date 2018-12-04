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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_ARENA_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth1/robot_arena_interactor.hpp"
#include "fordyca/support/depth2/cache_site_block_drop_interactor.hpp"
#include "fordyca/support/depth2/new_cache_block_drop_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
namespace ta = rcppsw::task_allocation;
class dynamic_cache_manager;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class robot_arena_interactor
 * @ingroup support depth2
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
 * - Creating a new cache.
 */
template <typename T>
class robot_arena_interactor : public depth1::robot_arena_interactor<T>,
                               public er::client<robot_arena_interactor<T>> {
 public:
  robot_arena_interactor(ds::arena_map* const map_in,
                         depth0::depth0_metrics_aggregator *const metrics_agg,
                         argos::CFloorEntity* const floor_in,
                         const ct::waveform_params* const block_manip_penalty,
                         const ct::waveform_params* const cache_usage_penalty,
                         dynamic_cache_manager* const cache_manager)
      : depth1::robot_arena_interactor<T>(map_in,
                                          metrics_agg,
                                          floor_in,
                                          block_manip_penalty,
                                          cache_usage_penalty),
    ER_CLIENT_INIT("fordyca.support.depth2.robot_arena_interactor"),
    m_cache_site_drop_interactor(map_in,
                                 floor_in,
                                 block_manip_penalty,
                                 cache_manager),
    m_new_cache_drop_interactor(map_in,
                                floor_in,
                                block_manip_penalty,
                                cache_manager) {}

  robot_arena_interactor& operator=(
      const robot_arena_interactor& other) = delete;
  robot_arena_interactor(const robot_arena_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   *
   * @return \c TRUE if a block was dropped to create a new cache, \c FALSE
   * otherwise.
   */
  bool operator()(T& controller, uint timestep) {
    std::list<temporal_penalty_handler<T>*> penalty_handlers =  {
      nest_drop_interactor().penalty_handler(),
      free_pickup_interactor().penalty_handler(),
      &cache_penalty_handler()
    };
    if (task_abort_interactor()(controller, penalty_handlers)) {
      return false;
    }

    if (controller.is_carrying_block()) {
      nest_drop_interactor()(controller, timestep);
      existing_cache_drop_interactor()(controller, timestep);
      m_cache_site_drop_interactor(controller, timestep);
      return m_new_cache_drop_interactor(controller, timestep);
    } else { /* The foot-bot has no block item */
      free_pickup_interactor()(controller, timestep);
      cached_pickup_interactor()(controller, timestep);
      return false;
    }
  }


 private:
  using depth1::robot_arena_interactor<T>::nest_drop_interactor;
  using depth1::robot_arena_interactor<T>::free_pickup_interactor;
  using depth1::robot_arena_interactor<T>::task_abort_interactor;
  using depth1::robot_arena_interactor<T>::cached_pickup_interactor;
  using depth1::robot_arena_interactor<T>::existing_cache_drop_interactor;
  using depth1::robot_arena_interactor<T>::cache_penalty_handler;

  // clang-format off
  cache_site_block_drop_interactor<T> m_cache_site_drop_interactor;
  new_cache_block_drop_interactor<T>  m_new_cache_drop_interactor;
  // clang-format on
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_ROBOT_ARENA_INTERACTOR_HPP_ */
