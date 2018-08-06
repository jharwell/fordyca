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
#include "fordyca/support/temporal_penalty_handler.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

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
class block_op_penalty_handler : public temporal_penalty_handler<T> {
 public:
  block_op_penalty_handler(std::shared_ptr<rcppsw::er::server> server,
                                 representation::arena_map* const map,
                                 const ct::waveform_params* const params)
      : temporal_penalty_handler<T>(server, params), m_map(map) {}

  ~block_op_penalty_handler(void) override = default;
  block_op_penalty_handler& operator=(
      const block_op_penalty_handler& other) = delete;
  block_op_penalty_handler(
      const block_op_penalty_handler& other) = delete;
  /**
   * @brief Check if a robot has acquired a block or is in the nest, and is
   * trying to drop/pickup a block. If so, create a \ref block_op_penalty object
   * and associate it with the robot.
   *
   * @param robot The robot to check.
   * @param timestep The current timestep.
   *
   * @return \c TRUE if a penalty has been initialized for a robot, and they
   * should begin waiting, and \c FALSE otherwise.
   */
  bool penalty_init(T& controller, uint timestep) {
    /* For nest block drops, this will always be unused and -1 */
    int block_id = utils::robot_on_block(controller, *m_map);

    /*
     * If the robot has not acquired a block, or thinks it has but actually has
     * not, nothing to do. If a robot is carrying a block but is still
     * transporting it (even if it IS currently in the nest), nothing to do.3
     */
    bool precond = false;
    if (controller.goal_acquired() &&
          acquisition_goal_type::kBlock == controller.acquisition_goal() &&
          -1 != block_id) {
      precond = true;
    }
    if (controller.in_nest() && controller.goal_acquired()) {
      precond = true;
    }
    if (!precond) {
      return precond;
    }

    ER_ASSERT(!temporal_penalty_handler<T>::is_serving_penalty(controller),
              "FATAL: Robot already serving block penalty?");

    uint penalty = temporal_penalty_handler<T>::deconflict_penalty_finish(timestep);
    ER_NOM("fb%d: start=%u, penalty=%u, adjusted penalty=%d",
           utils::robot_id(controller),
           timestep,
           temporal_penalty_handler<T>::original_penalty(),
           penalty);

    temporal_penalty_handler<T>::penalty_list().push_back(
        temporal_penalty<T>(&controller, block_id, penalty, timestep));
    return true;
  }

  // clang-format off
  representation::arena_map* const m_map;
  // clang-format on
};
NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_OP_PENALTY_HANDLER_HPP_ */
