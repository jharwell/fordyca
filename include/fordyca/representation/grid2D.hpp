/**
 * @file grid2D.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_GRID2D_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_GRID2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/multi_array.hpp>
#include <list>
#include <argos/core/utility/math/vector2.h>
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/params/params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief A 2D logical grid that is overlayed over the arena as part of a
 * representation of arena state (either global or per-robot).  It discretizes
 * the continuous arena into a grid of a specified resolution onto which blocks
 * and caches are mapped as points with an extent.
 *
 * Each cell within the 2D grid has its own state machine for reacting to
 */
class grid2D {
 public:
  explicit grid2D(
      const grid_params* params,
      const std::shared_ptr<rcppsw::common::er_server>& server =
      rcppsw::common::g_null_server);

  ~grid2D(void);

  static argos::CVector2 coord_to_cell(double x, double y);
  std::list<const cell2D*> with_blocks(void);
  cell2D& access(size_t i, size_t j) { return *m_cells[i][j]; }
  double resolution(void) const { return m_resolution; }
  size_t xsize(void) const { return std::ceil((m_upper.GetX() - m_lower.GetX()) / m_resolution); }
  size_t ysize(void) const { return std::ceil((m_upper.GetY() - m_lower.GetY()) / m_resolution); }

 private:
  double m_resolution;
  argos::CVector2 m_upper;
  argos::CVector2 m_lower;
  boost::multi_array<cell2D*, 2> m_cells;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_GRID2D_HPP_ */
