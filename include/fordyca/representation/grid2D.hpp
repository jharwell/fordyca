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
#include <algorithm>
#include <argos/core/utility/math/vector2.h>
#include "rcppsw/common/er_server.hpp"
#include "fordyca/params/params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
template<typename T>
using grid_type = typename boost::multi_array<T, 2>;
template<typename T>
using grid_view = typename grid_type<T>::template array_view<2>::type;
template<typename T>
using view_range = typename grid_type<T>::index_range;
using index_range = boost::multi_array_types::index_range;

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
 * events.
 */
template<typename T>
class grid2D {
 public:
  /* constructors/destructor */
  explicit grid2D(const grid_params* params,
                  const std::shared_ptr<rcppsw::common::er_server>& server =
                          rcppsw::common::g_null_server) :
      m_resolution(params->resolution),
      m_upper(params->upper),
      m_lower(params->lower),
      m_cells(boost::extents[xsize()][ysize()]) {
    for (auto i = m_cells.origin();
         i < m_cells.origin() + m_cells.num_elements(); ++i) {
      *i = new T(server);
    } /* for(i..) */
  }

  ~grid2D(void) {
    for (auto i = m_cells.origin();
         i < m_cells.origin() + m_cells.num_elements(); ++i) {
      delete *i;
    } /* for(i..) */
  }

  /* member functions */
  /**
   * @brief Create a subgrid (really an array view) from a grid. The grid is
   * clamped to the maximum boundaries of the parent grid, so rather than
   * getting a 2 x 2 subgrid centered at 0 with the out-of-bounds elements
   * zeroed, you will get a 1 x 2 subgrid.
   *
   * @param x X coord of center of subgrid.
   * @param y Y coord of center of subgrid.
   * @param radius Radius of subgrid.
   *
   * @return The subgrid.
   */
  grid_view<T*> subgrid(double x, double y, double radius) {
    size_t lower_x = std::max(0.0, x / m_resolution - radius / m_resolution);
    size_t upper_x = std::min(x / m_resolution + radius / m_resolution,
                              static_cast<double>(xsize()) - 1);
    size_t lower_y = std::max(0.0, y / m_resolution - radius / m_resolution);
    size_t upper_y = std::min(y / m_resolution + radius / m_resolution,
                              static_cast<double>(ysize()) - 1);
    if (lower_x > upper_x) {
      lower_x = upper_x - 1;
    }
    if (lower_y > upper_y) {
      lower_y = upper_y - 1;
    }
    typename grid_type<T*>::index_gen indices;
    return grid_view<T*>(m_cells[indices[index_range(lower_x, upper_x, 1)]
                                 [index_range(lower_y, upper_y, 1)]]);
  }
  T& access(size_t i, size_t j) { return *m_cells[i][j]; }
  double resolution(void) const { return m_resolution; }
  size_t xsize(void) const { return std::ceil((m_upper.GetX() - m_lower.GetX()) / m_resolution); }
  size_t ysize(void) const { return std::ceil((m_upper.GetY() - m_lower.GetY()) / m_resolution); }

 private:
  double m_resolution;
  argos::CVector2 m_upper;
  argos::CVector2 m_lower;
  grid_type<T*> m_cells;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_GRID2D_HPP_ */
