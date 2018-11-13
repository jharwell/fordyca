/**
 * @file utils.hpp
 * @ingroup math
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_MATH_UTILS_HPP_
#define INCLUDE_FORDYCA_MATH_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include "rcppsw/math/dcoord.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * namespaces
 ******************************************************************************/
NS_START(fordyca, math);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * @brief Translate real (continuous) coordinates to discrete ones using the
 * specified resolution.
 */
__rcsw_pure static inline rcppsw::math::dcoord2 rcoord_to_dcoord(
    const rmath::vector2d& r_coord,
    double resolution) {
  return rcppsw::math::dcoord2(
      static_cast<uint>(std::round(r_coord.x() / resolution)),
      static_cast<uint>(std::round(r_coord.y() / resolution)));
}

/**
 * @brief Translate discrete coordinates to real (continuous) ones using the
 * specified resolution.
 */
__rcsw_pure static inline rmath::vector2d dcoord_to_rcoord(
    const rcppsw::math::dcoord2& d_coord,
    double resolution) {
  return rmath::vector2d(d_coord.first * resolution,
                         d_coord.second * resolution);
}

NS_END(math, fordyca);

#endif /* INCLUDE_FORDYCA_MATH_UTILS_HPP_ */
