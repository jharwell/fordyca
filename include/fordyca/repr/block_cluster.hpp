/**
 * @file block_cluster.hpp
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

#ifndef INCLUDE_FORDYCA_REPR_BLOCK_CLUSTER_HPP_
#define INCLUDE_FORDYCA_REPR_BLOCK_CLUSTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/ds/block_list.hpp"
#include "fordyca/nsalias.hpp"
#include "fordyca/repr/grid_view_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_cluster
 * @ingroup fordyca repr
 *
 * @brief Represents a cluster of blocks in the arena as an entity for use
 * during block distribution and dynamic cache creation. A cluster is defined
 * as:
 *
 * - The 2D area in which the blocks reside
 * - The blocks distributed in that area.
 * - The maximum capacity of the cluster.
 */
class block_cluster final : public grid_view_entity<ds::arena_grid::const_view>,
                            public rer::client<block_cluster> {
 public:
  block_cluster(const ds::arena_grid::const_view& view,
                double resolution,
                uint capacity)
      : grid_view_entity<ds::arena_grid::const_view>(view, resolution),
        ER_CLIENT_INIT("fordyca.repr.block_cluster"),
        m_capacity(capacity) {}

  uint capacity(void) const { return m_capacity; }
  size_t block_count(void) const { return blocks().size(); }
  ds::const_block_list blocks(void) const;

 private:
  /* clang-format off */
  uint m_capacity;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPR_BLOCK_CLUSTER_HPP_ */
