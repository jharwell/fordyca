/**
 * @file block_cluster.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/block_cluster.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
uint block_cluster::block_count(void) const {
  uint count = 0;
  for (size_t i = 0; i < m_view.shape()[0]; ++i) {
    for (size_t j = 0; j < m_view.shape()[1]; ++j) {
      representation::cell2D* cell = m_view[i][j];
      assert(nullptr != cell);
      assert(!cell->state_has_cache());
      count += cell->state_has_block();
    }   /* for(j..) */
  }   /* for(i..) */
  return count;
} /* block_count() */

NS_END(representation, fordyca);
