/**
 * @file throttling_handler.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_THROTTLING_HANDLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_THROTTLING_HANDLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct throttling_params; }

NS_START(controller);
class actuation_subsystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class throttling_handler
 * @ingroup controller
 *
 * @brief Handler for all speed throttling for the robot.
 */
class throttling_handler {
 public:
  explicit throttling_handler(const struct params::throttling_params * params);

  /**
   * @brief Get the current amount of throttling (a percentage between 0 and 1)
   * that is configured for block carry.
   */
  double block_carry(void) const { return m_block_carry; }

  /**
   * @brief Set the current block carry state.
   */
  void carrying_block(bool carrying_block) { m_carrying_block = carrying_block; }

  /**
   * @brief Update the actuators in accordance with the current throttling
   * configuration.
   */
  void update(void);

 private:
  // clang-format off
  bool   m_carrying_block{false};
  double m_block_carry{0.0};
  double m_block_current{0.0};
  // clang-format off
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_THROTTLING_HANDLER_HPP_ */
