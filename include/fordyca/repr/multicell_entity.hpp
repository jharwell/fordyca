/**
 * @file multicell_entity.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_MULTICELL_ENTITY_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_MULTICELL_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "fordyca/repr/base_cell_entity.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/math/range.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class multicell_entity
 * @ingroup repr
 *
 * @brief Representation of an entity in the arena that can span multiple cells.
 *
 * All entities are assumed to be rectang in nature, or at least can be
 * reasonably approximated as such. All entities are assumed to maintain their
 * dimensions for their lifetime.
 */
class multicell_entity : public base_cell_entity {
 public:
  multicell_entity(const rmath::vector2d& dim, const rcppsw::utils::color& color)
      : multicell_entity{dim, color, -1} {}

  multicell_entity(const rmath::vector2d& dim,
                   const rcppsw::utils::color& color,
                   int id)
      : base_cell_entity(color, id), m_dim(dim) {}
  ~multicell_entity(void) override = default;

  /**
   * @brief Calculate the span in X of an entity given its location and
   * dimension in X.
   *
   * @return The span in X of the entity.
   */
  static rmath::ranged xspan(const rmath::vector2d& loc, double xdim) {
    return rmath::ranged(loc.x() - 0.5 * xdim, loc.x() + 0.5 * xdim);
  }

  /**
   * @brief Calculate the span in Y of an entity given its location and
   * dimension in Y.
   *
   * @return The span in Y of the entity.
   */
  static rmath::ranged yspan(const rmath::vector2d& loc, double ydim) {
    return rmath::ranged(loc.y() - 0.5 * ydim, loc.y() + 0.5 * ydim);
  }

  /**
   * @brief Get the 2D space spanned by the multicell entity in absolute
   * coordinates in the arena in Y. This is NOT the size of the entity in X.
   *
   * @param loc The entities current location.
   */
  rmath::ranged xspan(const rmath::vector2d& loc) const {
    return xspan(loc, m_dim.x());
  }

  /**
   * @brief Get the 2D space spanned by the multicell entity in absolute
   * coordinates in the arena in Y. This is NOT the size of the entity in Y.
   *
   * @param loc The entities current location.
   */

  rmath::ranged yspan(const rmath::vector2d& loc) const {
    return yspan(loc, m_dim.y());
  }

  /**
   * @brief Get the size of the cell entity in the X direction. This may be
   * greater than a single cell.
   */
  double xsize(void) const { return m_dim.x(); }

  /**
   * @brief Get the size of the cell entity in the y direction. This may be
   * greater than a single cell.
   */
  double ysize(void) const { return m_dim.y(); }

  const rmath::vector2d& dims(void) const { return m_dim; }
  double xdim(void) const { return m_dim.x(); }
  double ydim(void) const { return m_dim.y(); }

 private:
  /* clang-format off */
  rmath::vector2d m_dim;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_MULTICELL_ENTITY_HPP_ */
