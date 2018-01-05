/**
 * @file immovable_cell_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_IMMOVABLE_CELL_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_IMMOVABLE_CELL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/cell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class immovable_cell_entity
 * @ingroup representation
 *
 * @brief A base class from which objects whose locations in the arena do not
 * change during the lifetime of the object (e.g. caches) can derive from.
 */
class immovable_cell_entity : public cell_entity {
 public:
  /**
   * @param x_dim X dimension of the entity.
   * @param y_dim Y dimension of the entity.
   * @param color Color of the entity.
   * @param loc The entity's permant (for its lifetime) location in the arena.
   * @param resolution The resolution of the arena's discretization.
   */
  immovable_cell_entity(double x_dim, double y_dim, argos::CColor color,
                        const argos::CVector2& loc, double resolution) :
      cell_entity(x_dim, y_dim, color) {
    cell_entity::real_loc(loc);
    cell_entity::discrete_loc(representation::real_to_discrete_coord(loc,
                                                                     resolution));
  }

  immovable_cell_entity(double dim, argos::CColor color,
                        const argos::CVector2& loc, double resolution) :
      immovable_cell_entity(dim, dim, color, loc, resolution) {}
  ~immovable_cell_entity(void) override = default;


  immovable_cell_entity(const immovable_cell_entity& other) = default;
  immovable_cell_entity& operator=(const immovable_cell_entity& other) = default;

  const argos::CVector2& real_loc(void) const override { return cell_entity::real_loc(); }
  const discrete_coord& discrete_loc(void) const override { return cell_entity::discrete_loc(); }

 private:
  void discrete_loc(const discrete_coord&) override {}
  void real_loc(const argos::CVector2&) override {}
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_IMMOVABLE_CELL_ENTITY_HPP_ */
