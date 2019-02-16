/**
 * @file cluster_distributor.cpp
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
#include "fordyca/support/block_dist/cluster_distributor.hpp"
#include <algorithm>

#include "fordyca/ds/cell2D.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);
namespace er = rcppsw::er;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cluster_distributor::cluster_distributor(const ds::arena_grid::view& view,
                                         double arena_resolution,
                                         uint capacity)
    : base_distributor(),
      ER_CLIENT_INIT("fordyca.support.block_dist.cluster"),
      m_clust(view, capacity),
      m_dist(view, arena_resolution) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool cluster_distributor::distribute_block(
    std::shared_ptr<repr::base_block>& block,
    ds::const_entity_list& entities) {
  if (m_clust.capacity() == m_clust.block_count()) {
    ER_DEBUG("Could not distribute block%d: Cluster capacity (%u) reached",
             block->id(),
             m_clust.capacity());
    return false;
  }
  return m_dist.distribute_block(block, entities);
} /* distribute_block() */

bool cluster_distributor::distribute_blocks(ds::block_vector& blocks,
                                            ds::const_entity_list& entities) {
  if (m_clust.capacity() == m_clust.block_count()) {
    ER_DEBUG(
        "Could not distribute any of %zu blocks: Cluster capacity (%u) reached",
        blocks.size(),
        m_clust.capacity());
    return false;
  }
  return m_dist.distribute_blocks(blocks, entities);
} /* distribute_blocks() */

ds::const_block_cluster_list cluster_distributor::block_clusters(void) const {
  return ds::const_block_cluster_list{&m_clust};
} /* block_clusters() */

NS_END(block_dist, support, fordyca);
