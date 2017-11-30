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

#ifndef INCLUDE_FORDYCA_EXPRESSIONS_EXISTING_CACHE_UTILITY_HPP_
#define INCLUDE_FORDYCA_EXPRESSIONS_EXISTING_CACHE_UTILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/math/expression.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, expressions);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class existing_cache_utility
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
class existing_cache_utility: public rcppsw::math::expression<double> {
 public:
  existing_cache_utility(const argos::CVector2& cache_loc,
                         const argos::CVector2& nest_loc);

  double calc(const argos::CVector2& rloc, double density, size_t n_blocks);

 private:
  const argos::CVector2 mc_cache_loc;
  const argos::CVector2 mc_nest_loc;
};

NS_END(expressions, fordyca);

#endif /* INCLUDE_FORDYCA_EXPRESSIONS_EXISTING_CACHE_UTILITY_HPP_ */
