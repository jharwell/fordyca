/**
 * @file existing_cache_penalty_handler.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_EXISTING_CACHE_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_EXISTING_CACHE_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "fordyca/support/depth1/base_penalty_handler.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "fordyca/tasks/depth1/existing_cache_interactor.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "rcppsw/task_allocation/executable_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class existing_cache_penalty_handler
 * @ingroup support depth1
 *
 * @brief The handler for cache usage penalties for robots (i.e. how long they
 * have to wait when picking up/dropping in an existing cache).
 */
template <typename T>
class existing_cache_penalty_handler : public base_penalty_handler<T> {
 public:
  existing_cache_penalty_handler(const std::shared_ptr<rcppsw::er::server>&server,
                                 representation::arena_map* const map,
                        uint penalty)
      : base_penalty_handler<T>(server, penalty), m_map(map) {
    rcppsw::er::client::insmod("existing_cache_penalty_handler",
           rcppsw::er::er_lvl::DIAG,
           rcppsw::er::er_lvl::NOM);
  }

  ~existing_cache_penalty_handler(void) override = default;
  existing_cache_penalty_handler& operator=(
      const existing_cache_penalty_handler& other) = delete;
  existing_cache_penalty_handler(
      const existing_cache_penalty_handler& other) = delete;
  /**
   * @brief Check if a robot has acquired a cache, and is trying to pickup from
   * a cache, then creates a \ref block_manipulation_penalty object and
   * associates it with the robot.
   *
   * @param robot The robot to check.
   * @param timestep The current timestep.
   *
   * @return \c TRUE if a cache usage penalty has been initialized for a robot,
   * and they should begin waiting, and \c FALSE otherwise.
   */
  bool penalty_init(T& controller, uint timestep) {
    if (nullptr == controller.current_task()) {
      return false;
    } else if (nullptr == std::dynamic_pointer_cast<tasks::depth1::existing_cache_interactor>(
        controller.current_task())) {
      return false;
    } else if (!controller.current_task()->goal_acquired() ||
               !(acquisition_goal_type::kExistingCache ==
                 controller.current_task()->acquisition_goal())) {
      return false;
    }
    ER_ASSERT(acquisition_goal_type::kExistingCache ==
              controller.current_task()->acquisition_goal(),
              "FATAL: Goal for controller not existing cache");
    ER_ASSERT(controller.current_task()->goal_acquired(),
              "FATAL: Goal not acquired?");

    /* Check whether the foot-bot is actually on a cache */
    int cache_id = utils::robot_on_cache(controller, *m_map);
    if (-1 == cache_id) {
      return false;
    }

    ER_ASSERT(!controller.block_detected(),
              "FATAL: Block detected in cache?");
    ER_ASSERT(!base_penalty_handler<T>::is_serving_penalty(controller),
              "FATAL: Robot already serving cache penalty!");

    uint penalty = base_penalty_handler<T>::deconflict_penalty_finish(timestep);
    ER_NOM("fb%d: start=%u, penalty=%u, adjusted penalty=%d",
           utils::robot_id(controller),
           timestep,
           base_penalty_handler<T>::base_penalty(),
           penalty);

    base_penalty_handler<T>::penalty_list().push_back(
        block_manipulation_penalty<T>(&controller,
                                      cache_id,
                                      penalty,
                                      timestep));
    return true;
  }

  // clang-format off
  representation::arena_map* const m_map;
  // clang-format on
};
NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_EXISTING_CACHE_PENALTY_HANDLER_HPP_ */
