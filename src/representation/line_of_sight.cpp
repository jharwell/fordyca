/**
 * @file line_of_sight.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/cache.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<const representation::block*> line_of_sight::blocks(void) const {
  std::list<const representation::block*> blocks;

  for (size_t i = 0; i < m_view.shape()[0]; ++i) {
    for (size_t j = 0; j < m_view.shape()[1]; ++j) {
      representation::cell2D* cell = m_view[i][j];
      if (cell->state_has_block()) {
        assert(dynamic_cast<const representation::block*>(cell->block()));
        blocks.push_back(cell->block());
      }
    } /* for(j..) */
  } /* for(i..) */

  return blocks;
} /* blocks() */

std::list<const representation::cache*> line_of_sight::caches(void) const {
  std::list<const representation::cache*> caches;
  for (size_t i = 0; i < m_view.shape()[0]; ++i) {
    for (size_t j = 0; j < m_view.shape()[1]; ++j) {
      representation::cell2D* cell = m_view[i][j];
      if (cell->state_has_cache()) {
        assert(dynamic_cast<const representation::cache*>(cell->cache()));
        caches.push_back(cell->cache());
      }
    } /* for(j..) */
  } /* for(i..) */

  return caches;
} /* caches() */

__pure cell2D& line_of_sight::cell(size_t i, size_t j) const {
  assert(i < m_view.shape()[0]);
  assert(j < m_view.shape()[1]);
  return *m_view[i][j];
}

discrete_coord line_of_sight::cell_abs_coord(size_t i, size_t j) const {
  size_t abs_i_coord, abs_j_coord;
  if (i < sizex()/2) {
    abs_i_coord = m_center.first - i;
  } else {
    abs_i_coord = m_center.first + i;
  }
  if (j < sizey()/2) {
    abs_j_coord = m_center.second - j;
  } else {
    abs_j_coord = m_center.second + j;
  }
  assert(abs_i_coord > 0);
  assert(abs_j_coord > 0);
  return discrete_coord(abs_i_coord, abs_j_coord);
} /* cell_abs_coord() */

NS_END(representation, fordyca);
