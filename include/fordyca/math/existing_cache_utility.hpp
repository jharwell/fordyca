/**
 * \file existing_cache_utility.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/expression.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/repr/pheromone_density.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class existing_cache_utility
 * \ingroup math
 *
 * \brief Calculates the utility associated with an existing cache that the
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
class existing_cache_utility : public rmath::expression<double> {
 public:
  existing_cache_utility(const rmath::vector2d& cache_loc,
                         const rmath::vector2d& nest_loc);

  double calc(const rmath::vector2d& rloc,
              const crepr::pheromone_density& density,
              size_t n_blocks = 1);
  double operator()(const rmath::vector2d& rloc,
                    const crepr::pheromone_density& density,
                    size_t n_blocks) {
    return calc(rloc, density, n_blocks);
  }

 private:
  /* clang-format off */
  const rmath::vector2d mc_cache_loc;
  const rmath::vector2d mc_nest_loc;
  /* clang-format on */
};

NS_END(math, fordyca);
