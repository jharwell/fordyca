/**
 * @file immovable_cell_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_IMMOVABLE_CELL_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_IMMOVABLE_CELL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class immovable_cell_entity
 * @ingroup representation
 *
 * @brief A class representing objects that reside within one or more squares
 * within a 2D grid whose position CANNOT change during the lifetime of the
 * object.
 */
class immovable_cell_entity {
 public:
  /**
   * @brief Initialize a immovable entity with an initial location in the arena.
   */
  immovable_cell_entity(const rmath::vector2d& loc, double resolution)
      : m_real_loc(loc), m_discrete_loc(rmath::dvec2uvec(loc, resolution)) {}

  virtual ~immovable_cell_entity(void) = default;

  /**
   * @brief Get the real location (center) of the object.
   */
  const rmath::vector2d& real_loc(void) const { return m_real_loc; }

  /**
   * @brief Get the discretized coordinates of the center of the object, which
   * can be used to index into an arena_map.
   *
   */
  const rmath::vector2u& discrete_loc(void) const { return m_discrete_loc; }

 private:
  // clang-format off
  rmath::vector2d       m_real_loc;
  rmath::vector2u m_discrete_loc;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_IMMOVABLE_CELL_ENTITY_HPP_ */
