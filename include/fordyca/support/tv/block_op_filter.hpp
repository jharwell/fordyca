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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_FILTER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_FILTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acq_metrics.hpp"
#include "fordyca/support/tv/block_op_src.hpp"
#include "fordyca/support/utils/event_utils.hpp"
#include "fordyca/support/tv/op_filter_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, tv);

using acq_goal_type = metrics::fsm::goal_acq_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class block_op_filter
 * @ingroup fordyca support tv
 *
 * @brief The filter for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
template <typename T>
class block_op_filter : public rer::client<block_op_filter<T>> {
 public:
  explicit block_op_filter(const ds::arena_map* const map)
      : ER_CLIENT_INIT("fordyca.support.block_op_filter"), mc_map(map) {}

  ~block_op_filter(void) override = default;
  block_op_filter& operator=(const block_op_filter& other) = delete;
  block_op_filter(const block_op_filter& other) = delete;

  /**
   * @brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  op_filter_status operator()(T& controller,
                              block_op_src src,
                              rtypes::spatial_dist cache_prox) {
    /*
     * If the robot has not acquired a block, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a block but is still
     * transporting it (even if it IS currently in the nest), nothing to do.
     */
    switch (src) {
      case block_op_src::ekFREE_PICKUP:
        return free_pickup_filter(controller);
      case block_op_src::ekNEST_DROP:
        return nest_drop_filter(controller);
      case block_op_src::ekCACHE_SITE_DROP:
        return cache_site_drop_filter(controller,
                                      cache_prox);
      case block_op_src::ekNEW_CACHE_DROP:
        return new_cache_drop_filter(controller, cache_prox);
      default:
        ER_FATAL_SENTINEL("Unhandled penalty type %d", static_cast<int>(src));
    } /* switch() */
    return op_filter_status{};
  }

 private:
  /**
   * @brief Filter out spurious penalty initializations for free block pickup
   * (i.e. controller not ready/not intending to pickup a free block).
   *
   */
  op_filter_status free_pickup_filter(const T& controller) const {
    int block_id = utils::robot_on_block(controller, *mc_map);
    if (!(controller.goal_acquired() &&
          acq_goal_type::ekBLOCK == controller.acquisition_goal())) {
      return op_filter_status::ekROBOT_INTERNAL_UNREADY;
    } else if (-1 == block_id) {
      return op_filter_status::ekROBOT_NOT_ON_BLOCK;
    }
    return op_filter_status::ekSATISFIED;
  }

  /**
   * @brief Filter out spurious penalty initializations for nest block drop
   * (i.e. controller not ready/not intending to drop a block in the nest).
   */
  op_filter_status nest_drop_filter(const T& controller) const {
    if (!(controller.in_nest() && controller.goal_acquired() &&
          transport_goal_type::ekNEST == controller.block_transport_goal())) {
      return op_filter_status::ekROBOT_INTERNAL_UNREADY;
    }
    return op_filter_status::ekSATISFIED;
  }

  /**
   * @brief Filter out spurious penalty initializations for cache site drop
   * (i.e. controller not ready/not intending to drop a block), or another
   * block/cache is too close.
   */
  op_filter_status cache_site_drop_filter(const T& controller,
                                          rtypes::spatial_dist cache_prox) const {
    if (!(controller.goal_acquired() &&
          acq_goal_type::ekCACHE_SITE == controller.acquisition_goal() &&
          transport_goal_type::ekCACHE_SITE == controller.block_transport_goal())) {
      return op_filter_status::ekROBOT_INTERNAL_UNREADY;
    }

    int cache_id = utils::new_cache_cache_proximity(controller,
                                                    *mc_map,
                                                    cache_prox)
                   .entity_id;
    if (-1 != cache_id) {
      return op_filter_status::ekCACHE_PROXIMITY;
    }
    return op_filter_status::ekSATISFIED;
  }

  /**
   * @brief Filter out spurious penalty initializations for new cache drop
   * (i.e. controller not ready/not intending to drop a block), or
   * is too close to another cache to do a free block drop at the chosen site.
   */
  op_filter_status new_cache_drop_filter(const T& controller,
                                         rtypes::spatial_dist cache_prox) const {
    if (!(controller.goal_acquired() &&
          acq_goal_type::ekNEW_CACHE == controller.acquisition_goal() &&
          transport_goal_type::ekNEW_CACHE == controller.block_transport_goal())) {
      return op_filter_status::ekROBOT_INTERNAL_UNREADY;
    }
    int cache_id = utils::new_cache_cache_proximity(controller,
                                                    *mc_map,
                                                    cache_prox)
                   .entity_id;
    if (-1 != cache_id) {
      return op_filter_status::ekCACHE_PROXIMITY;
    }
    return op_filter_status::ekSATISFIED;
  }

  /* clang-format off */
  const ds::arena_map* const mc_map;
  /* clang-format on */
};
NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_FILTER_HPP_ */
