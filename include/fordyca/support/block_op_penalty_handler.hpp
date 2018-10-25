/**
 * @file block_op_penalty_handler.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <utility>

#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"
#include "fordyca/support/temporal_penalty_handler.hpp"

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
 * @class block_op_penalty_handler
 * @ingroup support
 *
 * @brief The handler for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
template <typename T>
class block_op_penalty_handler
    : public temporal_penalty_handler<T>,
      public er::client<block_op_penalty_handler<T>> {
 public:
  enum penalty_src {
    kSrcFreePickup,
    kSrcNestDrop,
    kSrcCacheSiteDrop,
    kSrcNewCacheDrop,
  };

  enum penalty_status {
    kStatusOK,
    kStatusControllerNotReady,
    kStatusControllerNotOnBlock,
    kStatusControllerNotInCache,
    kStatusBlockProximity,
    kStatusCacheProximity
  };
  using temporal_penalty_handler<T>::is_serving_penalty;
  using temporal_penalty_handler<T>::deconflict_penalty_finish;
  using temporal_penalty_handler<T>::original_penalty;

  block_op_penalty_handler(ds::arena_map* const map,
                           const ct::waveform_params* const params,
                           const std::string& name)
      : temporal_penalty_handler<T>(params, name),
        ER_CLIENT_INIT("fordyca.support.block_op_penalty_handler"),
        m_map(map) {}

  ~block_op_penalty_handler(void) override = default;
  block_op_penalty_handler& operator=(const block_op_penalty_handler& other) =
      delete;
  block_op_penalty_handler(const block_op_penalty_handler& other) = delete;

  /**
   * @brief Check if a robot has acquired a block or is in the nest, and is
   * trying to drop/pickup a block. If so, create a \ref block_op_penalty object
   * and associate it with the robot.
   *
   * @param robot The robot to check.
   * @param src The penalty source (i.e. what event caused this penalty to be
   *            applied).
   * @param timestep The current timestep.
   * @param cache_prox_dist The minimum distance that the cache site needs to
   *                        be from all caches in the arena.n
   *
   * @return \ref kStatusOK if a penalty has been initialized for a robot, a
   * non-zero code if it has not indicating why.
   */
  penalty_status penalty_init(T& controller,
                              penalty_src src,
                              uint timestep,
                              double cache_prox_dist = -1,
                              double block_prox_dist = -1) {
    auto filter = filter_controller(controller,
                                    src,
                                    cache_prox_dist,
                                    block_prox_dist);
    if (filter.first) {
      return filter.second;
    }
    ER_ASSERT(!is_serving_penalty(controller),
              "Robot already serving block penalty?");

    int id = penalty_id_calc(controller, src, cache_prox_dist, block_prox_dist);
    uint penalty = deconflict_penalty_finish(timestep);
    ER_INFO("fb%d: block%d start=%u, penalty=%u, adjusted penalty=%d src=%d",
            loop_utils::robot_id(controller),
            id,
            timestep,
            original_penalty(),
            penalty,
            src);

    penalty_list().push_back(
        temporal_penalty<T>(&controller, id, penalty, timestep));
    return kStatusOK;
  }

 protected:
  using temporal_penalty_handler<T>::penalty_list;
  using filter_status_type = std::pair<bool, penalty_status>;

 private:
  int penalty_id_calc(const T& controller,
                      penalty_src src,
                      double cache_prox_dist,
                      double block_prox_dist) const {
    int id = -1;
    switch (src) {
      case kSrcFreePickup:
        id = loop_utils::robot_on_block(controller, *m_map);
        ER_ASSERT(-1 != id, "Robot not on block?");
        break;
      case kSrcNestDrop:
      ER_ASSERT(nullptr != controller.block() && -1 != controller.block()->id(),
                "Robot not carrying block?");
      id = controller.block()->id();
      break;
      case kSrcCacheSiteDrop:
        ER_ASSERT(nullptr != controller.block() && -1 != controller.block()->id(),
                  "Robot not carrying block?");
        ER_ASSERT(block_prox_dist > 0.0,
                  "Block proximity distance not specified for cache site drop");
      id = controller.block()->id();
      break;
      case kSrcNewCacheDrop:
        ER_ASSERT(nullptr != controller.block() && -1 != controller.block()->id(),
                  "Robot not carrying block?");
        ER_ASSERT(cache_prox_dist > 0.0,
                  "Cache proximity distance not specified for new cache drop");
        id = controller.block()->id();
        break;
      default:
        id = -1;
    }
    return id;
  } /* penalty_id_calc() */

  /**
   * @brief Filters out controllers that actually are not eligible to start
   * serving penalties.
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_status_type filter_controller(T& controller,
                                       penalty_src src,
                                       double block_prox_dist,
                                       double cache_prox_dist) {
    /*
     * If the robot has not acquired a block, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a block but is still
     * transporting it (even if it IS currently in the nest), nothing to do.
     */
    filter_status_type res;
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
  }

  /**
   * @brief Filter out spurious penalty initializations for free block pickup
   * (i.e. controller not ready/not intending to pickup a free block).
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
 filter_status_type free_pickup_filter(const T& controller) const {
    int block_id = loop_utils::robot_on_block(controller, *m_map);
    if (!(controller.goal_acquired() &&
          acquisition_goal_type::kBlock == controller.acquisition_goal())) {
      return filter_status_type(true, kStatusControllerNotReady);
    } else if (-1 == block_id) {
      return filter_status_type(true, kStatusControllerNotOnBlock);
    }
    return filter_status_type(false, kStatusOK);
  }

  /**
   * @brief Filter out spurious penalty initializations for nest block drop
   * (i.e. controller not ready/not intending to drop a block in the nest).
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_status_type nest_drop_filter(const T& controller) const {
    if (!(controller.in_nest() && controller.goal_acquired())) {
      return filter_status_type(true, kStatusControllerNotReady);
    }
    return filter_status_type(false, kStatusOK);
  }

  /**
   * @brief Filter out spurious penalty initializations for cache site drop
   * (i.e. controller not ready/not intending to drop a block), or another block
   * is too close.
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_status_type cache_site_drop_filter(const T& controller,
                                            double block_prox_dist) const {
    int block_id = loop_utils::cache_site_block_proximity(controller,
                                                          *m_map,
                                                          block_prox_dist).first;
    if (!(controller.goal_acquired() &&
          acquisition_goal_type::kCacheSite == controller.acquisition_goal())) {
      return filter_status_type(true, kStatusControllerNotReady);
    }
    if (-1 != block_id) {
      return filter_status_type(true, kStatusBlockProximity);
    }
    return filter_status_type(false, kStatusOK);
  }

  /**
   * @brief Filter out spurious penalty initializations for new cache drop
   * (i.e. controller not ready/not intending to drop a block), or
   * is too close to another cache to do a free block drop at the chosen site.
   *
   * @return (\c TRUE, penalty_status) iff the controller should be filtered out
   * and the reason why. (\c FALSE, -1) otherwise.
   */
  filter_status_type new_cache_drop_filter(const T& controller,
                                           double cache_prox_dist) const {
    if (!(controller.goal_acquired() &&
          acquisition_goal_type::kNewCache == controller.acquisition_goal())) {
          return filter_status_type(true, kStatusControllerNotReady);
    }
    int cache_id = loop_utils::new_cache_cache_proximity(controller,
                                                         *m_map,
                                                         cache_prox_dist).first;
    if (-1 != cache_id) {
      return filter_status_type(true, kStatusCacheProximity);
    }
    return filter_status_type(false, kStatusOK);
  }

  // clang-format off
  ds::arena_map* const m_map;
  // clang-format on
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_PENALTY_HANDLER_HPP_ */
