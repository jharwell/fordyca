/**
 * @file cache_respawn_probability.hpp
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

#ifndef INCLUDE_FORDYCA_MATH_CACHE_RESPAWN_PROBABILITY_HPP_
#define INCLUDE_FORDYCA_MATH_CACHE_RESPAWN_PROBABILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "rcppsw/math/expression.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_respawn_probability
 * @ingroup math
 *
 * @brief Calculate the probability that loop functions should respawn a static
 * cache after it has been emptied (turned into a single block that is).
 *
 * Depends on:
 *
 * - A scaling factor > 0 that influences the probability distribution shape.
 */
class cache_respawn_probability : public rcppsw::math::expression<double> {
 public:
  explicit cache_respawn_probability(double scale_factor);

  /**
   * @brief Calculate the probability of respawn
   *
   * @param n_foragers # robots currently executing Forager task.
   * @param n_collectors # robots currently executing Collector task.
   */
  double calc(size_t n_foragers, size_t n_collectors);

 private:
  const double mc_scale_factor;
};

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_CACHE_RESPAWN_PROBABILITY_HPP_ */
