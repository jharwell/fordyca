/**
 * @file block_utility.hpp
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

#ifndef INCLUDE_FORDYCA_MATH_BLOCK_UTILITY_HPP_
#define INCLUDE_FORDYCA_MATH_BLOCK_UTILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/common/common.hpp"
#include "rcppsw/math/expression.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_utility
 * @ingroup math
 *
 * @brief Calculates the utility associated with a known block, as part of a
 * robot's decision on whether or not to go and attempt to pick it up.
 *
 * Depends on:
 *
 * - Distance of block to nest (Further is better).
 * - Distance of block to robot's current position (closer is better).
 * - Pheromone density associated with the block information (higher is better).
 */
class block_utility : public rcppsw::math::expression<double> {
 public:
  block_utility(const argos::CVector2& block_loc,
                const argos::CVector2& nest_loc);

  double calc(const argos::CVector2& rloc, double density, double priority);
  double operator()(const argos::CVector2& rloc,
                    double density,
                    double priority) {
    return calc(rloc, density, priority);
  }

 private:
  // clang-format off
  const argos::CVector2 mc_block_loc;
  const argos::CVector2 mc_nest_loc;
  // clang-format on
};

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_BLOCK_UTILITY_HPP_ */
