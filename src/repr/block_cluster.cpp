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
#include "fordyca/repr/block_cluster.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure ds::const_block_list block_cluster::blocks(void) const {
  ds::const_block_list ret;
  for (uint i = 0; i < xdimd(); ++i) {
    for (uint j = 0; j < ydimd(); ++j) {
      const auto& cell = block_cluster::cell(i, j);
      ER_ASSERT(!cell.state_has_cache(),
                "Cell@%s in HAS_CACHE state",
                cell.loc().to_str().c_str());
      ER_ASSERT(!cell.state_in_cache_extent(),
                "Cell@%s in CACHE_EXTENT state",
                cell.loc().to_str().c_str());
      if (cell.state_has_block()) {
        auto block = cell.block();
        ER_ASSERT(nullptr != block,
                  "Cell@%s null block",
                  cell.loc().to_str().c_str());
        ret.push_back(block);
      }
    } /* for(j..) */
  }   /* for(i..) */
  return ret;
} /* blocks() */

NS_END(repr, fordyca);
