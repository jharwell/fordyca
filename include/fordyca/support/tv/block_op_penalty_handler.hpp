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

#ifndef INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/support/tv/block_op_filter.hpp"
#include "fordyca/support/loop_utils/loop_utils.hpp"
#include "fordyca/support/tv/temporal_penalty_handler.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, tv);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class block_op_penalty_handler
 * @ingroup fordyca support tv
 *
 * @brief The handler for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
template <typename T>
class block_op_penalty_handler
    : public temporal_penalty_handler<T>,
      public rer::client<block_op_penalty_handler<T>> {
 public:
  using temporal_penalty_handler<T>::is_serving_penalty;
  using temporal_penalty_handler<T>::deconflict_penalty_finish;
  using temporal_penalty_handler<T>::original_penalty;

  block_op_penalty_handler(ds::arena_map* const map,
                           const rct::waveform_params* const params,
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
   * trying to drop/pickup a block. If so, create a \ref temporal_penalty object
   * and associate it with the robot.
   *
   * @param robot The robot to check.
   * @param src The penalty source (i.e. what event caused this penalty to be
   *            applied).
   * @param timestep The current timestep.
   * @param cache_prox_dist The minimum distance that the cache site needs to
   *                        be from all caches in the arena.n
   */
  op_filter_status penalty_init(T& controller,
                                block_op_src src,
                                uint timestep,
                                double cache_prox_dist = -1,
                                double block_prox_dist = -1) {
    auto filter = block_op_filter<T>(
        m_map)(controller, src, cache_prox_dist, block_prox_dist);
    if (filter.status) {
      return filter.reason;
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
    return op_filter_status::ekSATISFIED;
  }

 protected:
  using temporal_penalty_handler<T>::penalty_list;

 private:
  int penalty_id_calc(const T& controller,
                      block_op_src src,
                      double cache_prox_dist,
                      double block_prox_dist) const {
    int id = -1;
    switch (src) {
      case block_op_src::ekFREE_PICKUP:
        id = loop_utils::robot_on_block(controller, *m_map);
        ER_ASSERT(-1 != id, "Robot not on block?");
        break;
      case block_op_src::ekNEST_DROP:
        ER_ASSERT(nullptr != controller.block() &&
                      -1 != controller.block()->id(),
                  "Robot not carrying block?");
        id = controller.block()->id();
        break;
      case block_op_src::ekCACHE_SITE_DROP:
        ER_ASSERT(nullptr != controller.block() &&
                      -1 != controller.block()->id(),
                  "Robot not carrying block?");
        ER_ASSERT(block_prox_dist > 0.0,
                  "Block proximity distance not specified for cache site drop");
        id = controller.block()->id();
        break;
      case block_op_src::ekNEW_CACHE_DROP:
        ER_ASSERT(nullptr != controller.block() &&
                      -1 != controller.block()->id(),
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

  /* clang-format off */
  ds::arena_map* const m_map;
  /* clang-format on */
};
NS_END(tv, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_TV_BLOCK_OP_PENALTY_HANDLER_HPP_ */
