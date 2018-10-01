/**
 * @file multi_cluster_distributor.cpp
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
#include "fordyca/support/block_dist/multi_cluster_distributor.hpp"
#include <random>

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/representation/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);
namespace er = rcppsw::er;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
multi_cluster_distributor::multi_cluster_distributor(
    std::vector<ds::arena_grid::view>& grids,
    double arena_resolution,
    uint maxsize)
    : ER_CLIENT_INIT("fordyca.support.block_dist.multi_cluster") {
  for (size_t i = 0; i < grids.size(); ++i) {
    m_dists.emplace_back(grids[i],
                         arena_resolution,
                         maxsize);
  } /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool multi_cluster_distributor::distribute_block(
    std::shared_ptr<representation::base_block>& block,
    entity_list& entities) {

  for (size_t i = 0; i < kMAX_DIST_TRIES; ++i) {
    uint idx = std::rand() % m_dists.size();
    cluster_distributor& d = m_dists[idx];
    if (d.cluster().capacity() == d.cluster().block_count()) {
      ER_DEBUG("block%d to cluster%u failed: Cluster capacity (%u) reached",
               block->id(), idx, d.cluster().capacity());
    } else {
      return d.distribute_block(block, entities);
    }
  } /* for(i..) */
  return false;
} /* distribute_block() */

NS_END(block_dist, support, fordyca);
