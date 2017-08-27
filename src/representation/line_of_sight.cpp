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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<const representation::block*> line_of_sight::blocks(void) {
  std::list<const representation::block*> blocks;
  for (auto i = m_view.origin();
       i < m_view.origin() + m_view.num_elements(); ++i) {
    if ((*i)->state_has_block()) {
      blocks.push_back((*i)->block());
    }
  } /* for(i..) */
  return blocks;
} /* blocks() */

cell2D& line_of_sight::cell(size_t i, size_t j) const {
  assert(i < m_view.shape()[0]);
  assert(j < m_view.shape()[1]);
  return *m_view[i][j];
}

discrete_coord line_of_sight::cell_abs_coord(size_t i, size_t j) const {
  int abs_i_coord, abs_j_coord;
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
  return discrete_coord(std::min(std::max(0, abs_i_coord), (int)sizex() - 1),
                        std::min(std::max(0, abs_j_coord), (int)sizey() - 1));
} /* cell_abs_coord() */

NS_END(representation, fordyca);
