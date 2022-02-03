/**
 * \file cache_op_filter.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CACHE_OP_FILTER_HPP_
#define INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CACHE_OP_FILTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/argos/support/tv/cache_op_src.hpp"
#include "fordyca/argos/support/tv/op_filter_result.hpp"
#include "fordyca/fsm/foraging_acq_goal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class cache_op_filter
 * \ingroup argos support tv
 *
 * \brief The filter for cache operation for robots (e.g. picking up, dropping
 * in places that involve existing caches.
 */
class cache_op_filter : public rer::client<cache_op_filter> {
 public:
  explicit cache_op_filter(const carena::caching_arena_map* const map)
      : ER_CLIENT_INIT("fordyca.support.tv.cache_op_filter"), mc_map(map) {}

  ~cache_op_filter(void) override = default;
  cache_op_filter& operator=(const cache_op_filter&) = delete;
  cache_op_filter(const cache_op_filter&) = delete;

  /**
   * \brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   */
  template<typename TController>
  op_filter_result operator()(const TController& controller, cache_op_src src) {
    /*
     * If the robot has not acquired a cache, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a cache but is still
     * transporting it (even if it IS currently in the nest), nothing to do.
     */
    switch (src) {
      case cache_op_src::ekEXISTING_CACHE_DROP:
      case cache_op_src::ekEXISTING_CACHE_PICKUP:
        return do_filter(controller);
      default:
        ER_FATAL_SENTINEL("Unhandled penalty type %d", static_cast<int>(src));
    } /* switch() */
    return op_filter_result{};
  }

 private:
  /**
   * \brief Filter out spurious penalty initializations for existing cache
   * operations (e.g. pickup/drop) (i.e. controller not ready/not intending to
   * use an existing cache).
   */
  template <typename TController>
  op_filter_result do_filter(const TController& controller) const {
    op_filter_result result;
    if (!(controller.goal_acquired() &&
        fsm::foraging_acq_goal::ekEXISTING_CACHE ==
          controller.acquisition_goal())) {
      result.status = op_filter_status::ekROBOT_INTERNAL_UNREADY;
    } else {
      /*
       * Relatively few robots will have the correct internal state for a cache
       * operation each timestep, and we don't need exclusive access to the
       * arena map at this point--only a guarantee that the cache array will not
       * be modified while we are checking it.
       */
      mc_map->lock_rd(mc_map->cache_mtx());
      auto cache_id = mc_map->robot_on_cache(controller.rpos2D());
      mc_map->unlock_rd(mc_map->cache_mtx());

      if (rtypes::constants::kNoUUID != cache_id) {
        result.status = op_filter_status::ekSATISFIED;
        result.id = cache_id;
      }
    }
    return result;
  }

  /* clang-format off */
  const carena::caching_arena_map* mc_map;
  /* clang-format on */
};
NS_END(tv, support, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_SUPPORT_TV_CACHE_OP_FILTER_HPP_ */
