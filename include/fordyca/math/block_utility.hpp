/**
 * \file block_utility.hpp
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
 * \class block_utility
 * \ingroup math
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
