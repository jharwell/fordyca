/**
 * @file cluster_block_distributor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CLUSTER_BLOCK_DISTRIBUTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CLUSTER_BLOCK_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/random_block_distributor.hpp"
#include "fordyca/support/base_block_distributor.hpp"
#include "fordyca/representation/block_cluster.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
} // namespace representation

NS_START(support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cluster_block_distributor
 * @ingroup support
 *
 * @brief Distributes a block or set of blocks within the specified cluster
 * bounds randomly, using \ref random_block_distributor.
 */
class cluster_block_distributor : public base_block_distributor {
 public:
  cluster_block_distributor(std::shared_ptr<rcppsw::er::server> server,
                            representation::arena_grid::view& grid,
                            double arena_resolution,
                            uint maxsize);

  cluster_block_distributor& operator=(const cluster_block_distributor& s) = delete;

  bool distribute_block(std::shared_ptr<representation::block>& block,
                        entity_list& entities) override;
  bool distribute_blocks(block_vector& blocks, entity_list& entities) override;

  const representation:: block_cluster& cluster(void) const { return m_clust; }

 private:
  // clang-format off
  representation::block_cluster m_clust;
  random_block_distributor      m_dist;
  // clang-format on
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_CLUSTER_BLOCK_DISTRIBUTOR_HPP_ */
