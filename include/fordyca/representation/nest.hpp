/**
 * @file nest.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_NEST_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_NEST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "fordyca/representation/multicell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class nest
 * @ingroup representation
 *
 * @brief Class representing the nest in the arena, which is move multi-cellular
 * and immobile.
 */
class nest : public multicell_entity, public immovable_cell_entity {
 public:
  explicit nest(double xdim,
                double ydim,
                const argos::CVector2& loc,
                double resolution)
      : multicell_entity(xdim, ydim, rcppsw::utils::color::kGRAY70),
        immovable_cell_entity(loc, resolution) {}

  /**
   * @brief Determine if a real-valued point lies within the extent of the
   * nest for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot has entered the nest.
   *
   * @param point The point to check.
   *
   * @return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const argos::CVector2& point) const {
    return xspan(real_loc()).value_within(point.GetX()) &&
           yspan(real_loc()).value_within(point.GetY());
  }
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_NEST_HPP_ */
