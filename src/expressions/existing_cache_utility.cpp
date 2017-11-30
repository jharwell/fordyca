/**
 * @file existing_cache_utility.cpp
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
#include "fordyca/expressions/existing_cache_utility.hpp"
#include <cmath>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, expressions);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
existing_cache_utility::existing_cache_utility(const argos::CVector2& cache_loc,
                                                 const argos::CVector2& nest_loc) :
    mc_cache_loc(cache_loc), mc_nest_loc(nest_loc) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double existing_cache_utility::calc(const argos::CVector2& rloc,
                                    double density, size_t n_blocks) {
  return set_result((std::exp(-density) * n_blocks) /
                    ((mc_cache_loc - rloc).Length() *
                     (mc_cache_loc - mc_nest_loc).Length()));
} /* calc() */

NS_END(expressions, fordyca);
