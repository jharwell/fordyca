/**
 * @file block_selector.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_BLOCK_SELECTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_BLOCK_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <argos3/core/utility/math/vector2.h>

#include "rcppsw/er/client.hpp"
#include "fordyca/representation/perceived_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_selector
 * @ingroup controller depth0
 *
 * @brief Select the best block that a robot knows about, for use in acquiring a
 * block as part of a higher level FSM.
 */
class block_selector: public rcppsw::er::client {
 public:
  block_selector(
      const std::shared_ptr<rcppsw::er::server>& server,
      argos::CVector2 nest_loc);

  ~block_selector(void) override { rmmod(); }

  /**
   * @brief Given a list of blocks that a robot knows about (i.e. have not faded
   * into an unknown state), compute which is the "best", for use in deciding
   * which block to go attempt to pickup.
   *
   * @return A pointer to the "best" block, along with its utility value.
   */
  representation::perceived_block calc_best(
      const std::list<representation::perceived_block>& blocks,
      argos::CVector2 robot_loc);

 private:
  /**
   * @brief The minimum distance a robot has to be from a block for it to have a
   * non-zero utility. Allowing robots to consider ANY block, regardless of how
   * close it is to the robot, can potentially get the robot stuck in an
   * infinite loop of trying to acquire a block that is REALLY close to it and
   * failing, due to kinematic parameters making its turning radius too large.
   */
  static constexpr double kMinDist = 0.2;

  argos::CVector2 m_nest_loc;
};

NS_END(depth0, fordyca, controller);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH0_BLOCK_SELECTOR_HPP_ */
