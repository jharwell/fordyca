/**
 * @file cluster_distributor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_CLUSTER_DISTRIBUTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_CLUSTER_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/block_dist/random_distributor.hpp"
#include "fordyca/support/block_dist/base_distributor.hpp"
#include "fordyca/representation/block_cluster.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cluster_distributor
 * @ingroup support block_dist
 *
 * @brief Distributes a block or set of blocks within the specified cluster
 * bounds randomly, using \ref random_block_distributor.
 */
class cluster_distributor : public base_distributor,
                            public er::client<cluster_distributor> {
 public:
  cluster_distributor(const ds::arena_grid::view& view,
                      double arena_resolution,
                      uint capacity);
  ~cluster_distributor(void) override = default;

  cluster_distributor& operator=(const cluster_distributor& s) = delete;

  bool distribute_block(std::shared_ptr<representation::base_block>& block,
                        ds::const_entity_list& entities) override;
  bool distribute_blocks(ds::block_vector& blocks,
                         ds::const_entity_list& entities) override;

  ds::const_block_cluster_list block_clusters(void) const override;

 private:
  /* clang-format off */
  representation::block_cluster m_clust;
  random_distributor            m_dist;
  /* clang-format on */
};

NS_END(block_dist, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_CLUSTER_DISTRIBUTOR_HPP_ */
