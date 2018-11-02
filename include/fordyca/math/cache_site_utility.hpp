/**
 * @file cache_site_utility.hpp
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

#ifndef INCLUDE_FORDYCA_MATH_CACHE_SITE_UTILITY_HPP_
#define INCLUDE_FORDYCA_MATH_CACHE_SITE_UTILITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/math/sigmoid.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, math);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_site_utility
 * @ingroup math
 *
 * @brief Calculates the utility associated with a cache to a robot as part
 * of its decision process for what to do with a block once it has picked it up.
 *
 * Depends on:
 *
 * - Distance of perspective site to the nest (closer is better).
 * - Distance of perspective site to robot's current location (closer is
 * - better).
 *
 * The ideal location is halfway between the robot's current position and the
 * nest. However, if that location is unsuitable (i.e. it violates one or more
 * constraints), then we need to constrain our site selection to along the arc
 * of the circle formed by the nest center and distance to the ideal
 * point. We do this via exponential falloff on either side of the arc.
 */
class cache_site_utility : public rcppsw::math::sigmoid,
                           public rcppsw::er::client<cache_site_utility> {
 public:
  cache_site_utility(const argos::CVector2& robot_loc,
                     const argos::CVector2& nest_loc);

  double calc(const argos::CVector2& site_loc);
  double operator()(const argos::CVector2& site_loc) { return calc(site_loc); }

 private:
  // clang-format off
  const argos::CVector2 mc_robot_loc;
  const argos::CVector2 mc_nest_loc;
  // clang-format on
};

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_CACHE_SITE_UTILITY_HPP_ */
