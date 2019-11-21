/**
 * \file block_utility.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "rcppsw/math/expression.hpp"
#include "rcppsw/math/vector2.hpp"

#include "fordyca/fordyca.hpp"

#include "cosm/repr/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_utility
 * \ingroup fordyca math
 *
 * \brief Calculates the utility associated with a known block, as part of a
 * robot's decision on whether or not to go and attempt to pick it up.
 *
 * Depends on:
 *
 * - Distance of block to nest (Further is better).
 * - Distance of block to robot's current position (closer is better).
 * - Pheromone density associated with the block information (higher is better).
 * - Block priority of the block type being evaluated.
 */
class block_utility : public rmath::expression<double> {
 public:
  block_utility(const rmath::vector2d& block_loc,
                const rmath::vector2d& nest_loc);

  double calc(const rmath::vector2d& rloc,
              const crepr::pheromone_density& density,
              double priority);
  double operator()(const rmath::vector2d& rloc,
                    const crepr::pheromone_density& density,
                    double priority) {
    return calc(rloc, density, priority);
  }

 private:
  /* clang-format off */
  const rmath::vector2d mc_block_loc;
  const rmath::vector2d mc_nest_loc;
  /* clang-format on */
};

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_BLOCK_UTILITY_HPP_ */
