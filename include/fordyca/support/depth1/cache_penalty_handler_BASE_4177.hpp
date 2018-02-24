/**
 * @file cache_penalty_handler.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "fordyca/support/depth1/cache_penalty.hpp"
#include "fordyca/support/loop_functions_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class cache_penalty_handler
 * @ingroup support depth1
 *
 * @brief The handler for cache usage penalties for robots (i.e. how long they
 * have to wait).
 *
 * Handles:
 *
 * - Robots picking up from/dropping in a cache.
 * - Subjecting robots using caches to a penalty on both pickup/drop.
 */
class cache_penalty_handler : public rcppsw::er::client {
 public:
  cache_penalty_handler(const std::shared_ptr<rcppsw::er::server>&server,
                        std::shared_ptr<representation::arena_map>& map,
                        uint penalty)
      : client(server), mc_penalty(penalty), m_penalty_list(), m_map(map) {
    insmod("cache_penalty_handler",
           rcppsw::er::er_lvl::DIAG,
           rcppsw::er::er_lvl::NOM);
  }

  ~cache_penalty_handler(void) override { client::rmmod(); }

  /**
   * @brief Check if a robot has acquired a cache, and is trying to pickup from
   * a cache, then creates a \ref cache_penalty_penalty object and associates it
   * with the robot.
   *
   * @param robot The robot to check.
   * @param timestep The current timestep.
   *
   * @return \c TRUE if a cache usage penalty has been initialized for a robot,
   * and they should begin waiting, and \c FALSE otherwise.
   */
  template<typename T>
  bool penalty_init(argos::CFootBotEntity& robot,
                    uint timestep) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    if (controller.cache_acquired()) {
      /* Check whether the foot-bot is actually on a cache */
      int cache_id = utils::robot_on_cache(robot, m_map);
      if (-1 == cache_id) {
        return false;
      }

      ER_ASSERT(!controller.block_detected(),
                "FATAL: Block detected in cache?");
      ER_ASSERT(!is_serving_penalty<T>(robot),
                "FATAL: Robot already serving cache penalty");

      ER_NOM("fb%d: start=%u, duration=%u",
             utils::robot_id(robot),
             timestep,
             mc_penalty);
      uint penalty = mc_penalty;

      /*
       * Due to assertions in the \ref cache_block_pickup, if two robots enter
       * the cache at the EXACT same timestep, whichever one is processed second
       * for that timestep will trigger an assert due to the cache having a
       * different amount of blocks than it should (i.e. only one block pickup
       * is allowed per timestep, otherwise the world is inconsistent).
       *
       * Work around this by extending the penalty of the second robot to not
       * conflict with any other robots currently waiting/starting to wait.
       */
      for (auto it = m_penalty_list.begin(); it != m_penalty_list.end(); ++it) {
        if (it->start_time() + it->penalty() == timestep + penalty) {
          ++penalty;
          it = m_penalty_list.begin();
        }
      } /* for(i..) */

      m_penalty_list.push_back(cache_penalty(&controller,
                                             cache_id,
                                             penalty,
                                             timestep));
      return true;
    }
    return false;
  }

  /**
   * @brief Determine if a robot has satisfied the \ref cache_penalty_penalty yet.
   *
   * @param robot The robot to check. If the robot is not currently serving a
   * penalty, \c FALSE will be returned.
   * @param timestep The current timestep.
   *
   * @return \c TRUE If the robot is currently waiting AND it has satisfied its
   * penalty.
   */
  template<typename T>
  bool penalty_satisfied(argos::CFootBotEntity& robot,
                         uint timestep) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const cache_penalty& p) {
                             return p.controller() == &controller;});
    if (it != m_penalty_list.end()) {
      return it->penalty_satisfied(timestep);
    }
    return false;
  }
  cache_penalty& next(void) {return m_penalty_list.front(); }
  void remove(cache_penalty& victim) { return m_penalty_list.remove(victim); }

  template<typename T>
  void penalty_abort(argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    auto it = std::find_if(m_penalty_list.begin(),
                           m_penalty_list.end(),
                           [&](const cache_penalty& p) {
                             return p.controller() == &controller;
                           });
    if (it != m_penalty_list.end()) {
      m_penalty_list.remove(*it);
    }
    ER_NOM("fb%d",
           utils::robot_id(robot));
    ER_ASSERT(!is_serving_penalty<T>(robot),
              "FATAL: Robot still serving penalty after abort");
  }

  /**
   * @brief If \c TRUE, then the specified robot is currently serving a cache
   * penalty.
   */
  template<typename T>
  bool is_serving_penalty(
      argos::CFootBotEntity& robot) {
    auto& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const cache_penalty& p) {
                             return p.controller() == &controller; });
    return it != m_penalty_list.end();
  }


 private:
  // clang-format off
  uint                                       mc_penalty;
  std::list<cache_penalty>                   m_penalty_list;
  std::shared_ptr<representation::arena_map> m_map;
  // clang-format on
};
NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_HANDLER_HPP_ */
