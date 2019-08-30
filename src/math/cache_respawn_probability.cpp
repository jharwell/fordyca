/**
 * @file cache_respawn_probability.cpp
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
