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
    kFreePickup,
    kNestDrop,
    kCacheSiteDrop,
    kNewCacheDrop,
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
   *
   * @return \c TRUE if a penalty has been initialized for a robot, and they
   * should begin waiting, and \c FALSE otherwise.
   */
  bool penalty_init(T& controller,
                    penalty_src src,
                    uint timestep,
                    double cache_prox_dist = -1) {
    /*
     * If the robot has not acquired a block, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a block but is still
     * transporting it (even if it IS currently in the nest), nothing to do.3
     */
    if (kFreePickup == src && !free_pickup_filter(controller)) {
      return false;
    } else if (kNestDrop == src && !nest_drop_filter(controller)) {
      return false;
    } else if (kCacheSiteDrop == src && !cache_site_drop_filter(controller)) {
      return false;
    } else if (kNewCacheDrop == src &&
               !new_cache_drop_filter(controller, cache_prox_dist)) {
      return false;
    }
    ER_ASSERT(!is_serving_penalty(controller),
              "Robot already serving block penalty?");

    uint penalty = deconflict_penalty_finish(timestep);
    int id = -1;
    if (kFreePickup == src) {
      id = loop_utils::robot_on_block(controller, *m_map);
      ER_ASSERT(-1 != id, "Robot not on block?");
    } else if (kNestDrop == src) {
      ER_ASSERT(nullptr != controller.block() && -1 != controller.block()->id(),
                "Robot not carrying block?");
      id = controller.block()->id();
    } else if (kCacheSiteDrop == src) {
      ER_ASSERT(nullptr != controller.block() && -1 != controller.block()->id(),
                "Robot not carrying block?");
      ER_ASSERT(cache_prox_dist > 0.0,
                "Cache proximity distance not specified for cache site drop");
    }
    ER_INFO("fb%d: block%d start=%u, penalty=%u, adjusted penalty=%d src=%d",
            loop_utils::robot_id(controller),
            id,
            timestep,
            original_penalty(),
            penalty,
            src);

    penalty_list().push_back(
        temporal_penalty<T>(&controller, id, penalty, timestep));
    return true;
  }

  /**
   * @brief Filter out spurious penalty initializations for free block pickup
   * (i.e. controller not ready/not intending to pickup a free block).
   *
   * @return \c TRUE if the controller has met preconditions for free block
   * pickup, \c FALSE otherwise.
   */
  bool free_pickup_filter(const T& controller) const {
    int block_id = loop_utils::robot_on_block(controller, *m_map);
    return (controller.goal_acquired() &&
            acquisition_goal_type::kBlock == controller.acquisition_goal() &&
            -1 != block_id);
  }

  /**
   * @brief Filter out spurious penalty initializations for nest block drop
   * (i.e. controller not ready/not intending to drop a block in the nest).
   *
   * @return \c TRUE if the controller has met preconditions for nest block
   * drop, \c FALSE otherwise.
   */
  bool nest_drop_filter(const T& controller) const {
    return (controller.in_nest() && controller.goal_acquired());
  }

  /**
   * @brief Filter out spurious penalty initializations for cache site drop
   * (i.e. controller not ready/not intending to drop a block), or
   * is too close to a block to do a free block drop at the chosen site.
   *
   * @return \c TRUE if the controller has met preconditions for cache site
   * drop, \c FALSE otherwise.
   */
  bool cache_site_drop_filter(const T& controller) const {
    int block_id =
        loop_utils::cache_site_block_proximity(controller, *m_map).first;
    return (controller.goal_acquired() &&
            acquisition_goal_type::kCacheSite == controller.acquisition_goal() &&
            -1 == block_id);
  }
  /**
   * @brief Filter out spurious penalty initializations for new cache drop
   * (i.e. controller not ready/not intending to drop a block), or
   * is too close to another cache to do a free block drop at the chosen site.
   *
   * @return \c TRUE if the controller has met preconditions for cache site
   * drop, \c FALSE otherwise.
   */
  bool new_cache_drop_filter(const T& controller, double cache_prox_dist) const {
    int cache_id = loop_utils::new_cache_cache_proximity(controller,
                                                         *m_map,
                                                         cache_prox_dist)
                       .first;
    return (controller.goal_acquired() &&
            acquisition_goal_type::kCacheSite == controller.acquisition_goal() &&
            -1 == cache_id);
  }

 protected:
  using temporal_penalty_handler<T>::penalty_list;

 private:
  // clang-format off
  ds::arena_map* const m_map;
  // clang-format on
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_PENALTY_HANDLER_HPP_ */
