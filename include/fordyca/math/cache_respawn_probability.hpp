/**
 * \file cache_respawn_probability.hpp
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

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_respawn_probability
 * \ingroup math
 *
 * \brief Calculate the probability that loop functions should respawn a static
 * cache after it has been emptied (turned into a single block that is).
 *
 * Depends on:
 *
 * - A scaling factor > 0 that influences the probability distribution shape.
 */
class cache_respawn_probability : public rmath::expression<double> {
 public:
  explicit cache_respawn_probability(double scale_factor);

  /**
   * \brief Calculate the probability of respawn
   *
   * \param n_harvesters # robots currently executing Harvester task.
   * \param n_collectors # robots currently executing Collector task.
   */
  double calc(uint n_harvesters, uint n_collectors);

 private:
  const double mc_scale_factor;
};

NS_END(math, fordyca);
