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

#ifndef INCLUDE_FORDYCA_EXPRESSIONS_CACHE_RESPAWN_PROBABILITY_HPP_
#define INCLUDE_FORDYCA_EXPRESSIONS_CACHE_RESPAWN_PROBABILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <cmath>

#include "rcppsw/math/expression.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, expressions);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_respawn_probability: public rcppsw::math::expression<double>  {
 public:
  explicit cache_respawn_probability(double scale_factor) :
      mc_scale_factor(scale_factor) {}

  double calc(size_t n_foragers, size_t n_collectors) {
    return set_result(1 - std::exp(mc_scale_factor * n_foragers/n_collectors));
  }

 private:
  const double mc_scale_factor;
};

NS_END(expressions, fordyca);

#endif /* INCLUDE_FORDYCA_EXPRESSIONS_CACHE_RESPAWN_PROBABILITY_HPP_ */
