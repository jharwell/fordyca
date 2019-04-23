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
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/support/tv/cache_op_src.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, tv);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;
namespace er = rcppsw::er;

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
class cache_op_filter : public er::client<cache_op_filter<T>> {
 public:
  enum filter_status {
    kStatusOK,
    kStatusControllerNotReady,
  };
  /**
   * @brief The result of checking a controller instance to see if it has
   * satisfied the preconditions for a cache operation (pickup/drop/etc).
   *
   * If \c TRUE, then the controller should be filtered out and has NOT
   * satisfied the preconditions. In this case, the reason is set to indicate
   * why it failed the preconditions.
   *
   * If \c FALSE, then the controller should NOT be filtered out, and has
   * satisfied the preconditions. reason is undefined in this case.
   */
  struct filter_res_t {
    bool status;
    filter_status reason;
  };

  explicit cache_op_filter(ds::arena_map* const map)
      : ER_CLIENT_INIT("fordyca.support.cache_op_filter"), m_map(map) {}

  ~cache_op_filter(void) override = default;
  cache_op_filter& operator=(const cache_op_filter& other) = delete;
  cache_op_filter(const cache_op_filter& other) = delete;

  /**
   * @brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   *
   * @return (\c TRUE, status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_res_t operator()(T& controller, cache_op_src src) {
    /*
     * If the robot has not acquired a cache, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a cache but is still
     * transporting it (even if it IS currently in the nest), nothing to do.
     */
    switch (src) {
      case kSrcExistingCacheDrop:
      case kSrcExistingCachePickup:
        return do_filter(controller);
      default:
        ER_FATAL_SENTINEL("Unhandled penalty type %d", src);
    } /* switch() */
    ER_FATAL_SENTINEL("Unhandled penalty type %d", src);
    return filter_res_t{};
  }

 private:
  /**
   * @brief Filter out spurious penalty initializations for existing cache
   * operations (e.g. pickup/drop) (i.e. controller not ready/not intending to
   * use an existing cache).
   *
   */
  filter_res_t do_filter(const T& controller) const {
    int cache_id = loop_utils::robot_on_cache(controller, *m_map);
    bool ready = (controller.goal_acquired() &&
                  acquisition_goal_type::kExistingCache ==
                      controller.acquisition_goal() &&
                  -1 != cache_id);
    if (ready) {
      return filter_res_t{false, kStatusOK};
    }
    return filter_res_t{true, kStatusControllerNotReady};
  }

  /* clang-format off */
  ds::arena_map* const m_map;
  /* clang-format on */
};
NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_CACHE_OP_FILTER_HPP_ */
