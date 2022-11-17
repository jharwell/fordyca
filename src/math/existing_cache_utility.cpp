/**
 * \file existing_cache_utility.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/math/existing_cache_utility.hpp"

#include <cmath>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
existing_cache_utility::existing_cache_utility(const rmath::vector2d& cache_loc,
                                               const rmath::vector2d& nest_loc)
    : mc_cache_loc(cache_loc), mc_nest_loc(nest_loc) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double existing_cache_utility::calc(const rmath::vector2d& rloc,
                                    const crepr::pheromone_density& density,
                                    size_t n_blocks) {
  return eval(
      (std::exp(density.v()) * n_blocks) /
      ((mc_cache_loc - rloc).length() * (mc_cache_loc - mc_nest_loc).length()));
} /* calc() */

NS_END(expressions, fordyca);
