/**
 * @file cache_op_filter.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_CACHE_OP_FILTER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_CACHE_OP_FILTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/fsm/block_transporter.hpp"
#include "cosm/fsm/metrics/goal_acq_metrics.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "fordyca/support/utils/event_utils.hpp"
#include "fordyca/support/tv/op_filter_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, tv);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class cache_op_filter
 * @ingroup fordyca support tv
 *
 * @brief The filter for cache operation for robots (e.g. picking up, dropping
 * in places that involve existing caches.
 */
template <typename T>
class cache_op_filter : public rer::client<cache_op_filter<T>> {
 public:
  explicit cache_op_filter(const ds::arena_map* const map)
      : ER_CLIENT_INIT("fordyca.support.cache_op_filter"), mc_map(map) {}

  ~cache_op_filter(void) override = default;
  cache_op_filter& operator=(const cache_op_filter& other) = delete;
  cache_op_filter(const cache_op_filter& other) = delete;

  /**
   * @brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   */
  op_filter_status operator()(T& controller, cache_op_src src) {
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
    return op_filter_status{};
  }

 private:
  /**
   * @brief Filter out spurious penalty initializations for existing cache
   * operations (e.g. pickup/drop) (i.e. controller not ready/not intending to
   * use an existing cache).
   *
   */
  op_filter_status do_filter(const T& controller) const {
    int cache_id = utils::robot_on_cache(controller, *mc_map);
    bool ready = (controller.goal_acquired() &&
                  fsm::foraging_acq_goal::ekEXISTING_CACHE ==
                      controller.acquisition_goal() &&
                  -1 != cache_id);
    if (ready) {
      return op_filter_status::ekSATISFIED;
    }
    return op_filter_status::ekROBOT_INTERNAL_UNREADY;
  }

  /* clang-format off */
  const ds::arena_map* const mc_map;
  /* clang-format on */
};
NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_CACHE_OP_FILTER_HPP_ */
