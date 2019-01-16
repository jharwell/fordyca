/**
 * @file cache_op_penalty_handler.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CACHE_OP_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CACHE_OP_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/support/cache_op_filter.hpp"
#include "fordyca/support/cache_op_src.hpp"
#include "fordyca/support/temporal_penalty_handler.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller { namespace depth0 {
class stateful_foraging_controller;
class crw_controller;
}} // namespace controller::depth0

NS_START(support);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class cache_op_penalty_handler
 * @ingroup support
 *
 * @brief The handler for block operation penalties for robots (e.g. picking
 * up, dropping in places that do not involve existing caches.
 */
template <typename T>
class cache_op_penalty_handler
    : public temporal_penalty_handler<T>,
      public er::client<cache_op_penalty_handler<T>> {
 public:
  using temporal_penalty_handler<T>::is_serving_penalty;
  using temporal_penalty_handler<T>::deconflict_penalty_finish;
  using temporal_penalty_handler<T>::original_penalty;

  cache_op_penalty_handler(ds::arena_map* const map,
                           const ct::waveform_params* const params,
                           const std::string& name)
      : temporal_penalty_handler<T>(params, name),
        ER_CLIENT_INIT("fordyca.support.cache_op_penalty_handler"),
        m_map(map) {}

  ~cache_op_penalty_handler(void) override = default;
  cache_op_penalty_handler& operator=(const cache_op_penalty_handler& other) =
      delete;
  cache_op_penalty_handler(const cache_op_penalty_handler& other) = delete;

  using filter_status = typename cache_op_filter<T>::filter_status;

  /**
   * @brief Check if a robot has acquired a block or is in the nest, and is
   * trying to drop/pickup a block. If so, create a \ref cache_op_penalty object
   * and associate it with the robot.
   *
   * @param robot The robot to check.
   * @param src The penalty source (i.e. what event caused this penalty to be
   *            applied).
   * @param timestep The current timestep.
   *
   * @return \c TRUE if a penalty has been initialized for a robot, and they
   * should begin waiting, and \c FALSE otherwise, along with the reason why.
   */
  filter_status penalty_init(T& controller, cache_op_src src, uint timestep) {
    /*
     * If the robot has not acquired a cache, or thinks it has but actually has
     * not, nothing to do.
     */
    auto filter = cache_op_filter<T>(m_map)(controller, src);
    if (filter.status) {
      return filter.reason;
    }

    ER_ASSERT(!is_serving_penalty(controller),
              "Robot already serving cache penalty?");

    uint penalty = deconflict_penalty_finish(timestep);
    int id = loop_utils::robot_on_cache(controller, *m_map);
    ER_ASSERT(-1 != id, "Robot not in cache?");
    ER_INFO("fb%d: cache%d start=%u, penalty=%u, adjusted penalty=%d src=%d",
            loop_utils::robot_id(controller),
            id,
            timestep,
            original_penalty(),
            penalty,
            src);

    penalty_list().push_back(
        temporal_penalty<T>(&controller, id, penalty, timestep));
    return filter_status::kStatusOK;
  }

 protected:
  using temporal_penalty_handler<T>::penalty_list;

 private:
  /* clang-format off */
  ds::arena_map* const m_map;
  /* clang-format on */
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CACHE_OP_PENALTY_HANDLER_HPP_ */
