/**
 * \file cache_respawn_probability.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/math/cache_respawn_probability.hpp"

#include <cmath>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_respawn_probability::cache_respawn_probability(double scale_factor)
    : mc_scale_factor(scale_factor) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double cache_respawn_probability::calc(uint n_harvesters, uint n_collectors) {
  double tmp;
  if (0 == n_collectors) {
    tmp = mc_scale_factor * n_harvesters;
  } else {
    tmp = mc_scale_factor * n_harvesters / n_collectors;
  }
  return eval(1 - std::exp(-tmp));
} /* calc() */

NS_END(expressions, fordyca);
