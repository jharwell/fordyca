/**
 * @file block_manipulation_penalty.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_BLOCK_MANIPULATION_PENALTY_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_BLOCK_MANIPULATION_PENALTY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_manipulation_penalty
 * @ingroup support depth1
 *
 * @brief Handles subjecting a robot to a penalty when picking up/dropping a
 * block, whether that is in a cache or just in the arena via a specified
 * timeout in which the robot will sit still.
 */
template<typename T>
class block_manipulation_penalty {
 public:
  /**
   * @brief Initialize the penalty.
   *
   * @param controller The controller for the robot being subjected to the
   * penalty.
   * @param id An optional ID to associate with the penalty.
   * @param penalty The # of timesteps for the penalty.
   * @param start_time The timestep the penalty will start on.
   */
  block_manipulation_penalty(const T* const controller,
                             uint id,
                             uint penalty,
                             uint start_time)
      : mc_id(id),
        mc_penalty(penalty),
        mc_start_time(start_time),
        mc_controller(controller) {}

  const T* controller(void) const { return mc_controller; }
  uint start_time(void) const { return mc_start_time; }
  uint penalty(void) const { return mc_penalty; }
  uint id(void) const { return mc_id; }

  bool operator==(const block_manipulation_penalty& other) {
    return this->controller() == other.controller();
  }

  /**
   * @brief If \c TRUE, then the robot has satisfied the block_manipulation
   * penalty.
   */
  bool penalty_satisfied(uint current_time) const {
    return current_time - mc_start_time >= mc_penalty;
  }

 private:
  // clang-format off
  uint          mc_id;
  uint          mc_penalty;
  uint          mc_start_time;
  const T*const mc_controller;
  // clang-format on
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_BLOCK_MANIPULATION_PENALTY_HPP_ */
