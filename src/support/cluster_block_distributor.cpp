/**
 * @file cluster_block_distributor.cpp
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
#include "fordyca/support/cluster_block_distributor.hpp"
#include <algorithm>

#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
namespace er = rcppsw::er;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cluster_block_distributor::cluster_block_distributor(
    std::shared_ptr<rcppsw::er::server> server,
    representation::arena_grid::view& grid,
    double arena_resolution,
    uint maxsize)
    : base_block_distributor(server),
      m_clust(grid, maxsize),
      m_dist(server, grid, arena_resolution) {
  if (ERROR == client::attmod("cluster_block_dist")) {
    insmod("cluster_block_dist", er::er_lvl::DIAG, er::er_lvl::VER);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool cluster_block_distributor::distribute_block(
    std::shared_ptr<representation::block>& block,
    entity_list& entities) {
  if (m_clust.capacity() == m_clust.block_count()) {
    ER_DIAG("Could not distribute block: Cluster capacity (%u) reached",
            m_clust.capacity());
    return false;
  }
  return m_dist.distribute_block(block, entities);
} /* distribute_block() */

bool cluster_block_distributor::distribute_blocks(
    block_vector& blocks,
    entity_list& entities) {
  if (m_clust.capacity() == m_clust.block_count()) {
    ER_DIAG("Could not distribute block: Cluster capacity (%u) reached",
            m_clust.capacity());
    return false;
  }
  return m_dist.distribute_blocks(blocks, entities);
} /* distribute_blocks() */

NS_END(support, fordyca);
