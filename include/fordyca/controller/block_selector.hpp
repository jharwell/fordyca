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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BLOCK_SELECTOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BLOCK_SELECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/ds/block_list.hpp"
#include "fordyca/representation/perceived_block.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace rmath = rcppsw::math;

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
class block_selector : public rcppsw::er::client<block_selector> {
 public:
  explicit block_selector(const block_sel_matrix* sel_matrix);

  ~block_selector(void) override = default;

  block_selector& operator=(const block_selector& other) = delete;
  block_selector(const block_selector& other) = delete;
  /**
   * @brief Given a list of blocks that a robot knows about (i.e. have not faded
   * into an unknown state), compute which is the "best", for use in deciding
   * which block to go attempt to pickup.
   *
   * @return A pointer to the "best" block, along with its utility value.
   */
  representation::perceived_block calc_best(const ds::perceived_block_list& blocks,
                                            const rmath::vector2d& position);

 private:
  bool block_is_excluded(const rmath::vector2d& position,
                         const representation::base_block* block) const;

  /**
   * @brief The minimum distance a robot has to be from a block for it to have a
   * non-zero utility. Allowing robots to consider ANY block, regardless of how
   * close it is to the robot, can potentially get the robot stuck in an
   * infinite loop of trying to acquire a block that is REALLY close to it and
   * failing, due to kinematic parameters making its turning radius too large.
   */
  static constexpr double kMinDist = 0.2;

  // clang-format off
  const block_sel_matrix* const mc_matrix;
  // clang-format on
};

NS_END(fordyca, controller);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BLOCK_SELECTOR_HPP_ */
