/**
 * @file cache_penalty.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller { namespace depth1 { class foraging_controller; }}

NS_START(support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_penalty
 * @ingroup support depth1
 *
 * @brief Handles subjecting a robot to a penalty when picking up from/dropping
 * in a cache via a specified timeout in which the robot will sit still.
 */
class cache_penalty {
 public:
  cache_penalty(const controller::depth1::foraging_controller* const controller,
                      uint cache_id,
                      uint penalty,
                      uint start_time) : m_cache_id(cache_id),
                                         m_penalty(penalty),
                                         m_start_time(start_time),
                                         mc_controller(controller) {}

  uint cache_id(void) const { return m_cache_id; }
  const controller::depth1::foraging_controller* controller(void) const { return mc_controller; }
  uint start_time(void) const { return m_start_time; }
  uint penalty(void) const { return m_penalty; }

  bool operator==(const cache_penalty& other) {
    return this->controller() == other.controller();
  }

  /**
   * @brief If \c TRUE, then the robot has satisfied the cache penalty.
   */
  bool penalty_satisfied(uint current_time) {
    return current_time - m_start_time >= m_penalty;
  }

 private:
  // clang-format off
  uint                                                m_cache_id;
  uint                                                m_penalty;
  uint                                                m_start_time;
  const controller::depth1::foraging_controller*const mc_controller;
  // clang-format on
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_CACHE_PENALTY_HPP_ */
