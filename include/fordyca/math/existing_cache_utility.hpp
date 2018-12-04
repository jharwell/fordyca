/**
 * @file existing_cache_utility.hpp
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

#ifndef INCLUDE_FORDYCA_MATH_EXISTING_CACHE_UTILITY_HPP_
#define INCLUDE_FORDYCA_MATH_EXISTING_CACHE_UTILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/math/expression.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class existing_cache_utility
 * @ingroup math
 *
 * @brief Calculates the utility associated with an existing cache that the
 * robot knows about.
 *
 * Depends on:
 *
 * - Distance of cache to nest (closer is better).
 * - Distance of cache to robot's current position (closer is better).
 * - # of blocks believed to be in the cache (more is better).
 * - Pheromone density associated with the cache information (higher is
 *   better).
 */
class existing_cache_utility : public rcppsw::math::expression<double> {
 public:
  existing_cache_utility(const rmath::vector2d& cache_loc,
                         const rmath::vector2d& nest_loc);

  double calc(const rmath::vector2d& rloc, double density, size_t n_blocks = 1);
  double operator()(const rmath::vector2d& rloc,
                    double density,
                    size_t n_blocks) {
    return calc(rloc, density, n_blocks);
  }

 private:
  const rmath::vector2d mc_cache_loc;
  const rmath::vector2d mc_nest_loc;
};

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_EXISTING_CACHE_UTILITY_HPP_ */
