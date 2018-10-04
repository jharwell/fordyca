/**
 * @file arena_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_ARENA_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth1/arena_interactor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
namespace ta = rcppsw::task_allocation;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class arena_interactor
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
class arena_interactor : public depth1::arena_interactor<T>,
                         public er::client<arena_interactor<T>> {
 public:
  arena_interactor(ds::arena_map* const map_in,
                   depth0::stateless_metrics_aggregator *const metrics_agg,
                   argos::CFloorEntity* const floor_in,
                   const ct::waveform_params* const block_manip_penalty,
                   const ct::waveform_params* const cache_usage_penalty)
      : depth1::arena_interactor<T>(map_in,
                                    metrics_agg,
                                    floor_in,
                                    block_manip_penalty,
                                    cache_usage_penalty),
    ER_CLIENT_INIT("fordyca.support.depth2.arena_interactor") {}

  arena_interactor& operator=(const arena_interactor& other) = delete;
  arena_interactor(const arena_interactor& other) = delete;

  /**
   * @brief The actual handling function for interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep   The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    if (handle_task_abort(controller)) {
      return;
    }
    if (controller.is_carrying_block()) {
      handle_nest_block_drop(controller, timestep);
      if (cache_penalty_handler().is_serving_penalty(controller)) {
        if (cache_penalty_handler().penalty_satisfied(controller,
                                                      timestep)) {
          finish_cache_block_drop(controller);
        }
      } else {
        cache_penalty_handler().penalty_init(controller,
                                             penalty_type::kExistingCacheDrop,
                                             timestep);
      }

    } else { /* The foot-bot has no block item */
      handle_free_block_pickup(controller, timestep);
      if (cache_penalty_handler().is_serving_penalty(controller)) {
        if (cache_penalty_handler().penalty_satisfied(controller,
                                                      timestep)) {
          finish_cached_block_pickup(controller, timestep);
        }
      } else {
        cache_penalty_handler().penalty_init(controller,
                                             penalty_type::kExistingCachePickup,
                                             timestep);
      }
    }
  }

 protected:
  using depth1::arena_interactor<T>::cache_penalty_handler;
  using depth1::arena_interactor<T>::handle_task_abort;
  using depth1::arena_interactor<T>::handle_nest_block_drop;
  using depth1::arena_interactor<T>::finish_cache_block_drop;
  using depth1::arena_interactor<T>::finish_cached_block_pickup;
  using depth1::arena_interactor<T>::handle_free_block_pickup;
  using penalty_type = typename depth1::arena_interactor<T>::penalty_type;

 private:
  // clang-format off
  // clang-format on
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_ARENA_INTERACTOR_HPP_ */
