/**
 * @file foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include "fordyca/support/depth0/stateful_foraging_loop_functions.hpp"
#include "fordyca/support/depth1/cache_usage_penalty.hpp"
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/events/cached_block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace metrics { namespace collectors {
class task_collector;
namespace robot_metrics {
class depth1_metrics_collector;
}}}
namespace robot_collectors = metrics::collectors::robot_metrics;

NS_START(support, depth1);

class cache_usage_penalty;

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class foraging_loop_functions
 *
 * @brief The loop functions for depth 1 foraging.
 *
 * Handles:
 *
 * - Robots picking up from/dropping in a cache
 * - Subjecting robots using caches to a penalty (only on pickup).
 */
class foraging_loop_functions : public depth0::stateful_foraging_loop_functions {
 public:
  foraging_loop_functions(void);
  virtual ~foraging_loop_functions(void);

  void Init(argos::TConfigurationNode& node) override;
  void PreStep() override;
  void Destroy(void) override;
  void Reset(void) override;

 protected:
  /**
   * @brief Check if a robot has acquired a cache, and is trying to pickup from
   * a cache, then creates a \ref cache_usage_penalty object and associates it
   * with the robot.
   */
  template<typename T>
  bool init_cache_usage_penalty(
      argos::CFootBotEntity& robot) {
    T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    if (controller.cache_acquired() && !controller.is_transporting_to_cache()) {
      ER_ASSERT(!controller.block_detected(), "FATAL: Block detected in cache?");

      /* Check whether the foot-bot is actually on a cache */
      int cache = utils::robot_on_cache(robot, *map());
      if (-1 != cache) {
        m_penalty_list.push_back(new cache_usage_penalty(&controller,
                                                         cache,
                                                         mc_cache_penalty,
                                                         GetSpace().GetSimulationClock()));
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Determine if a robot has satisfied the \ref cache_usage_penalty yet.
   */
  template<typename T>
  bool cache_usage_penalty_satisfied(argos::CFootBotEntity& robot) {

    T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const cache_usage_penalty* p) {
                             return p->controller() == &controller;});
    if (it != m_penalty_list.end()) {
      return (*it)->penalty_satisfied(GetSpace().GetSimulationClock());
    }

    return false;
  }

  /**
   * @brief Called after a robot has satisfied the cache usage penalty, and
   * actually performs the handshaking between the cache, the arena, and the
   * robot for block pickup.
   */
  template<typename T>
  void finish_cached_block_pickup(argos::CFootBotEntity& robot) {

    T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    cache_usage_penalty* p = m_penalty_list.front();
    ER_ASSERT(p->controller() == &controller,
              "FATAL: Out of order cache penalty handling");

    events::cached_block_pickup pickup_op(rcppsw::er::g_server,
                                          &map()->caches()[p->cache_id()],
                                          utils::robot_id(robot));
    m_penalty_list.remove(p);

    /*
     * Map must be called before controller for proper cache block decrement!
     */
    map()->accept(pickup_op);

    controller.visitor::template visitable_any<T>::accept(pickup_op);
    floor()->SetChanged();
  }

  /**
   * @brief If \c TRUE, then the specified robot is currently serving a cache
   * penalty.
   */
  template<typename T>
  bool robot_serving_cache_penalty(
      argos::CFootBotEntity& robot) {
    T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());
    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const cache_usage_penalty* p) {
                             return p->controller() == &controller;});
    return it != m_penalty_list.end();
  }

  /**
   * @brief Handles handshaking between cache, robot, and arena if the robot is
   * has acquired a cache and is looking to drop an object in it.
   */
  template<typename T>
  bool handle_cache_block_drop(argos::CFootBotEntity& robot) {
    T& controller = static_cast<T&>(robot.GetControllableEntity().GetController());

    if (controller.cache_acquired() && controller.is_transporting_to_cache()) {
      /* Check whether the foot-bot is actually on a cache */
      int cache = utils::robot_on_cache(robot, *map());
      if (-1 != cache) {
        /* Update arena map state due to a block nest drop */
        events::cache_block_drop drop_op(rcppsw::er::g_server,
                                         controller.block(),
                                         &map()->caches()[cache],
                                         map()->grid_resolution());

        map()->accept(drop_op);
        controller.visitor::template visitable_any<T>::accept(drop_op);
        return true;
      }
    }
    return false;
  }

 private:
  void pre_step_final(void) override;
  void pre_step_iter(argos::CFootBotEntity& robot);
  argos::CColor GetFloorColor(const argos::CVector2& plane_pos) override;

  foraging_loop_functions(const foraging_loop_functions& s) = delete;
  foraging_loop_functions& operator=(const foraging_loop_functions& s) = delete;

  uint                                                        mc_cache_penalty;
  double                                                      mc_cache_respawn_scale_factor;
  std::unique_ptr<robot_collectors::depth1_metrics_collector> m_depth1_collector;
  std::unique_ptr<metrics::collectors::task_collector>        m_task_collector;
  std::list<cache_usage_penalty*>                             m_penalty_list;
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_FORAGING_LOOP_FUNCTIONS_HPP_ */
