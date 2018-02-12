/**
 * @file sub_area_poa.hpp
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

#ifndef INCLUDE_FORDYCA_MATH_SUB_AREA_POA_HPP_
#define INCLUDE_FORDYCA_MATH_SUB_AREA_POA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
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
 * @brief Calculates the probability that a particular sub-area will be of
 * interest to a robot.
 *
 * Depends on:
 *
 * - Sub area distance to nest. This emphasizes sub areas that are further away,
 *   encouraging bringing items from further away before bring items that are
 *   closer, facilitating a general movement of items closer to the nest.
 *
 * - Total distance from all known caches in the subarea to nest. This
 *   emphasizes exploiting existing caches when they exist, rather than go
 *   exploring.
 *
 * - TODO: take pheromones into account?
 */
class sub_area_poa : public rcppsw::math::expression<double> {
 public:
  sub_area_poa(const argos::CVector2& area_center,
               const argos::CVector2& nest_center)
      : mc_center(area_center), mc_nest(nest_center) {}

  double calc(double caches_dist) {
    return set_result((mc_center - mc_nest).Length() / caches_dist);
  }

 private:
  const argos::CVector2 mc_center;
  const argos::CVector2 mc_nest;
};

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_SUB_AREA_POA_HPP_ */
