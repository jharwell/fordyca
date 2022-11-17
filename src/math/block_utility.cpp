/**
 * \file block_utility.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/math/block_utility.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_utility::block_utility(const rmath::vector2d& block_loc,
                             const rmath::vector2d& nest_loc)
    : mc_block_loc(block_loc), mc_nest_loc(nest_loc) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double block_utility::calc(const rmath::vector2d& rloc,
                           const crepr::pheromone_density& density,
                           double priority) {
  return eval(
      ((mc_block_loc - mc_nest_loc).length() / (mc_block_loc - rloc).length()) *
      std::exp(density.v() * priority));
} /* calc() */

NS_END(expressions, fordyca);
