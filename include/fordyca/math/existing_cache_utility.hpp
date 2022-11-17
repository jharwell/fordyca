/**
 * \file existing_cache_utility.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
