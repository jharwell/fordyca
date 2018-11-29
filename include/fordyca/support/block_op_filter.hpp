/**
 * @file block_op_filter.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_FILTER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_FILTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"
#include "fordyca/support/block_op_src.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;
namespace er = rcppsw::er;

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class block_op_filter
 * @ingroup support
 *
 * @brief The filter for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
template <typename T>
class block_op_filter : public er::client<block_op_filter<T>> {
 public:
  enum filter_status {
    kStatusOK,
    kStatusControllerNotReady,
    kStatusControllerNotOnBlock,
    kStatusControllerNotInCache,
    kStatusBlockProximity,
    kStatusCacheProximity
  };
  /**
   * @brief The result of checking a controller instance to see if it has
   * satisfied the preconditions for a block operation (pickup/drop/etc).
   *
   * If \c TRUE, then the controller should be filtered out and has NOT
   * satisfied the preconditions. In this case, the reason is set to indicate
   * why it failed the preconditions.
   *
   * If \c FALSE, then the controller should NOT be filtered out, and has
   * satisfied the preconditions. reason is undefined in this case.
   */
  struct filter_result {
    bool status;
    filter_status reason;
  };

  explicit block_op_filter(ds::arena_map* const map)
      : ER_CLIENT_INIT("fordyca.support.block_op_filter"),
        m_map(map) {}

  ~block_op_filter(void) override = default;
  block_op_filter& operator=(const block_op_filter& other) =
      delete;
  block_op_filter(const block_op_filter& other) = delete;

  /**
   * @brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_result operator()(T& controller,
                           block_op_src src,
                           double cache_prox_dist,
                           double block_prox_dist) {
    /*
     * If the robot has not acquired a block, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a block but is still
     * transporting it (even if it IS currently in the nest), nothing to do.
     */
    switch (src) {
      case kSrcFreePickup:
        return free_pickup_filter(controller);
      case kSrcNestDrop:
        return nest_drop_filter(controller);
      case kSrcCacheSiteDrop:
        return cache_site_drop_filter(controller, block_prox_dist);
      case kSrcNewCacheDrop:
        return new_cache_drop_filter(controller, cache_prox_dist);
      default:
        ER_FATAL_SENTINEL("Unhandled penalty type %d", src);
    } /* switch() */
    ER_FATAL_SENTINEL("Unhandled penalty type %d", src);
    return filter_result{};
  }

 private:
  /**
   * @brief Filter out spurious penalty initializations for free block pickup
   * (i.e. controller not ready/not intending to pickup a free block).
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_result free_pickup_filter(const T& controller) const {
    int block_id = loop_utils::robot_on_block(controller, *m_map);
    if (!(controller.goal_acquired() &&
          acquisition_goal_type::kBlock == controller.acquisition_goal())) {
      return filter_result{true, kStatusControllerNotReady};
    } else if (-1 == block_id) {
      return filter_result{true, kStatusControllerNotOnBlock};
    }
    return filter_result{false, kStatusOK};
  }

  /**
   * @brief Filter out spurious penalty initializations for nest block drop
   * (i.e. controller not ready/not intending to drop a block in the nest).
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_result nest_drop_filter(const T& controller) const {
    if (!(controller.in_nest() && controller.goal_acquired() &&
          transport_goal_type::kNest == controller.block_transport_goal())) {
      return filter_result{true, kStatusControllerNotReady};
    }
    return filter_result{false, kStatusOK};
  }

  /**
   * @brief Filter out spurious penalty initializations for cache site drop
   * (i.e. controller not ready/not intending to drop a block), or another block
   * is too close.
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_result cache_site_drop_filter(const T& controller,
                                            double block_prox_dist) const {
    if (!(controller.goal_acquired() &&
          acquisition_goal_type::kCacheSite == controller.acquisition_goal() &&
          transport_goal_type::kCacheSite == controller.block_transport_goal())) {
      return filter_result{true, kStatusControllerNotReady};
    }
    int block_id = loop_utils::cache_site_block_proximity(controller,
                                                          *m_map,
                                                          block_prox_dist)
                       .entity_id;
    if (-1 != block_id) {
      return filter_result{true, kStatusBlockProximity};
    }
    return filter_result{false, kStatusOK};
  }

  /**
   * @brief Filter out spurious penalty initializations for new cache drop
   * (i.e. controller not ready/not intending to drop a block), or
   * is too close to another cache to do a free block drop at the chosen site.
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_result new_cache_drop_filter(const T& controller,
                                           double cache_prox_dist) const {
    if (!(controller.goal_acquired() &&
          acquisition_goal_type::kNewCache == controller.acquisition_goal() &&
          transport_goal_type::kNewCache == controller.block_transport_goal())) {
      return filter_result{true, kStatusControllerNotReady};
    }
    int cache_id = loop_utils::new_cache_cache_proximity(controller,
                                                         *m_map,
                                                         cache_prox_dist)
                       .entity_id;
    if (-1 != cache_id) {
      return filter_result{true, kStatusCacheProximity};
    }
    return filter_result{false, kStatusOK};
  }

  // clang-format off
  ds::arena_map* const m_map;
  // clang-format on
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_FILTER_HPP_ */