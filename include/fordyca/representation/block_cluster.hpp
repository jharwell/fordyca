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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_BLOCK_CLUSTER_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_BLOCK_CLUSTER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/ds/arena_grid.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_cluster
 * @ingroup representation
 *
 * @brief Represents a cluster of blocks in the arena as an entity for use
 * during block distribution.
 */
class block_cluster : public rcppsw::er::client<block_cluster> {
 public:
  block_cluster(const ds::arena_grid::const_view& view, uint capacity)
      : ER_CLIENT_INIT("fordyca.representation.block_cluster"),
        m_view(view),
        m_capacity(capacity) {}

  uint capacity(void) const { return m_capacity; }
  uint block_count(void) const;
  const ds::arena_grid::const_view& view(void) const { return m_view; }

 private:
  // clang-format off
  ds::arena_grid::const_view m_view;
  uint                       m_capacity;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_BLOCK_CLUSTER_HPP_ */
