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
#include <argos3/core/utility/math/vector2.h>

#include "fordyca/representation/base_cell_entity.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/math/range.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class multicell_entity
 * @ingroup representation
 *
 * @brief Representation of an entity in the arena that can span multiple cells.
 *
 * All entities are assumed to be rectang in nature, or at least can be
 * reasonably approximated as such. All entities are assumed to maintain their
 * dimensions for their lifetime.
 */
class multicell_entity : public base_cell_entity {
 public:
  multicell_entity(double xdim,
                   double ydim,
                   const rcppsw::utils::color& color)
      : multicell_entity{xdim, ydim, color, -1} {}

  multicell_entity(double dim,
                       const rcppsw::utils::color& color)
      : multicell_entity{dim, dim, color, -1} {}

  multicell_entity(double dim,
                       const rcppsw::utils::color& color,
                       int id)
      : multicell_entity{dim, dim, color, id} {}

  multicell_entity(double xdim,
                       double ydim,
                       const rcppsw::utils::color& color,
                       int id)
      : base_cell_entity(color, id), m_xdim(xdim), m_ydim(ydim) {}

  /**
   * @brief Get the 2D space spanned by the multicell entity in absolute
   * coordinates in the arena in Y. This is NOT the size of the entity in X.
   *
   * @param loc The entities current location.
   */
  rcppsw::math::range<double> xspan(const argos::CVector2& loc) const {
    return rcppsw::math::range<double>(loc.GetX() - 0.5 * m_xdim,
                                       loc.GetX() + 0.5 * m_xdim);
  }

  /**
   * @brief Get the 2D space spanned by the multicell entity in absolute
   * coordinates in the arena in Y. This is NOT the size of the entity in Y.
   *
   * @param loc The entities current location.
   */

  rcppsw::math::range<double> yspan(const argos::CVector2& loc) const {
    return rcppsw::math::range<double>(loc.GetY() - 0.5 * m_ydim,
                                       loc.GetY() + 0.5 * m_ydim);
  }

  /**
   * @brief Get the size of the cell entity in the X direction. This may be
   * greater than a single cell.
   */
  double xsize(void) const { return m_xdim; }

  /**
   * @brief Get the size of the cell entity in the y direction. This may be
   * greater than a single cell.
   */
  double ysize(void) const { return m_ydim; }

 private:
  double m_xdim;
  double m_ydim;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_MULTICELL_ENTITY_HPP_ */
