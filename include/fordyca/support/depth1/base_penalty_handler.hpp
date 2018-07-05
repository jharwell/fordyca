/**
 * @file base_penalty_handler.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_BASE_PENALTY_HANDLER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_BASE_PENALTY_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth1/block_manipulation_penalty.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "rcppsw/er/client.hpp"
#include "fordyca/support/depth1/cache_penalty_generator.hpp"
#include "fordyca/params/depth1/penalty_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class base_penalty_handler
 * @ingroup support depth1
 *
 * @brief The base handler for block manipulation penalties for robots (i.e. how
 * long they have to wait when they pickup/drop a block).
 *
 * Does not do much more than provide the penalty list, and functions for
 * manipulating it to derived classes.
 *
 *The handler for cache usage penalties for robots (i.e. how long they
 * have to wait).
 *
 * Handles:
 *
 * - Robots picking up from/dropping in a cache.
 * - Subjecting robots using caches to a penalty on both pickup/drop.
 */
template <typename T>
class base_penalty_handler : public rcppsw::er::client {
 public:
  /**
   * @Brief Initialize the penalty handler.
   *
   * @param server Server for debugging.
   * @param penalty The minimum penalty robots will serve (can be more if it is
   * adjusted to maintain simulation consistency).
   */
  base_penalty_handler(const std::shared_ptr<rcppsw::er::server>&server,
                        uint penalty,
                        const params::penalty_function pen_func,
                        int amp, int per, int phase, int square, int step,
                        int saw)
      : client(server), mc_penalty(penalty), m_penalty_list(),
      m_cache_penalty_generator(pen_func, amp, per, phase, square, step, saw)  {
    insmod("base_penalty_handler",
           rcppsw::er::er_lvl::DIAG,
           rcppsw::er::er_lvl::NOM);
  }

  ~base_penalty_handler(void) override { client::rmmod(); }

  uint base_penalty(void) const { return mc_penalty; }

  /**
   * @brief Determine if a robot has satisfied the \ref block_manipulation_penalty
   * it is currently serving yet.
   *
   * @param robot The robot to check. If the robot is not currently serving a
   * penalty, \c FALSE will be returned.
   * @param timestep The current timestep.
   *
   * @return \c TRUE If the robot is currently waiting AND it has satisfied its
   * penalty.
   */
  bool penalty_satisfied(const T& controller, uint timestep) const {
    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const block_manipulation_penalty<T>& p) {
                             return p.controller() == &controller;
                           });
    if (it != m_penalty_list.end()) {
      return it->penalty_satisfied(timestep);
    }
    return false;
  }
  /**
   * @brief Get the next robot that will satisfy its penalty from the list.
   */
  const block_manipulation_penalty<T>& next(void) const { return m_penalty_list.front(); }

  /**
   * @brief Remove the specified penalty from the list once the robot it
   * corresponds to has served its penalty.
   *
   * @param victim The penalty to remove.
   */
  void remove(const block_manipulation_penalty<T>& victim) {
    return m_penalty_list.remove(victim);
  }

  /**
   * @brief Abort a robot's serving of its penalty.
   *
   * This should only be done if the robot aborts its task WHILE also serving a
   * penalty.
   *
   * @param controller The robot to abort the penalty for.
   */
  void penalty_abort(const T& controller) {
    auto it = std::find_if(m_penalty_list.begin(),
                           m_penalty_list.end(),
                           [&](const block_manipulation_penalty<T>& p) {
                             return p.controller() == &controller;
                           });
    if (it != m_penalty_list.end()) {
      m_penalty_list.remove(*it);
    }
    ER_NOM("fb%d", utils::robot_id(controller));
    ER_ASSERT(!is_serving_penalty(controller),
              "FATAL: Robot still serving penalty after abort?!");
  }

  /**
   * @brief If \c TRUE, then the specified robot is currently serving a cache
   * penalty.
   */
  bool is_serving_penalty(const T& controller) const {
    auto it = std::find_if(m_penalty_list.begin(), m_penalty_list.end(),
                           [&](const block_manipulation_penalty<T>& p) {
                             return p.controller() == &controller; });
    return it != m_penalty_list.end();
  }

 protected:
  std::list<block_manipulation_penalty<T>>& penalty_list(void) {
    return m_penalty_list;
  }
  const std::list<block_manipulation_penalty<T>>& penalty_list(void) const {
    return m_penalty_list;
  }

  /*
   * @brief Deconflict cache penalties such that at most 1 robot finishes
   * serving their penalty in the cache per timestep. This ensures consistency
   * and proper processing of \ref cached_block_pickup events.

   * When handling \ref cache_block_pickup events, if two robots enter the cache
   * at the EXACT same timestep, whichever one is processed second for that
   * timestep will trigger an assert due to the cache having a different amount
   * of blocks than it should (i.e. only one block pickup is allowed per
   * timestep, otherwise the world is inconsistent).
   */
  uint deconflict_penalty_finish(uint timestep) const {
    uint penalty = (m_cache_penalty_generator.*penalty_func)(timestep);
    for (auto it = m_penalty_list.begin(); it != m_penalty_list.end(); ++it) {
      if (it->start_time() + it->penalty() == timestep + penalty) {
        ++penalty;
        it = m_penalty_list.begin();
      }
    } /* for(i..) */
    return penalty;
  }

  // clang-format off
  uint                                       mc_penalty;
  std::list<block_manipulation_penalty<T>>   m_penalty_list;
  cache_penalty_generator    m_cache_penalty_generator;
  // clang-format on
};
NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_BASE_PENALTY_HANDLER_HPP_ */
