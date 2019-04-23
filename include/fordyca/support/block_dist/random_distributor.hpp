/**
 * @file random_distributor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_RANDOM_DISTRIBUTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_RANDOM_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <vector>
#include <random>

#include "fordyca/nsalias.hpp"
#include "rcppsw/math/vector2.hpp"
#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/support/block_dist/base_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace repr {
class multicell_entity;
} // namespace repr

NS_START(support, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class random_distributor
 * @ingroup fordyca support
 *
 * @brief Distributes a set of blocks randomly within a specified 2D area, such
 * that no blocks overlap with each other or other entities already present in
 * the arena (nest, cache, etc.).
 *
 * - Blocks are assumed to be square (this is not checked).
 * - The dimension of the a block is assumed to be the same as the resolution of
 *   the arena map the blocks are being distributed into some part of (this is
 *   not checked).
 */
class random_distributor : public base_distributor,
                           public rer::client<random_distributor> {
 public:
  random_distributor(const ds::arena_grid::view& grid,
                     double resolution);

  random_distributor& operator=(const random_distributor& s) = delete;

  bool distribute_blocks(ds::block_vector& blocks,
                         ds::const_entity_list& entities) override;

  bool distribute_block(std::shared_ptr<repr::base_block>& block,
                        ds::const_entity_list& entities) override;
  ds::const_block_cluster_list block_clusters(void) const override {
    return ds::const_block_cluster_list();
  }

 private:
  struct coord_search_res_t {
    bool            status{false};
    rmath::vector2u rel{};
    rmath::vector2u abs{};
  };
  /**
   * @brief The maxmimum # of times the distribution will be attempted before
   * giving up.
   */
  static constexpr uint kMAX_DIST_TRIES = 100;

  /**
   * @brief Find coordinates for distribution that are outside the extent of the
   * all specified entities, while also accounting for block size.
   *
   * @param entities The entities to avoid.
   */
  coord_search_res_t avail_coord_search(const ds::const_entity_list& entities,
                                         const rmath::vector2d& block_dim);
  bool verify_block_dist(const repr::base_block* block,
                         const ds::const_entity_list& entities,
                         const ds::cell2D* cell);

  /* clang-format off */
  double               m_resolution;
  ds::arena_grid::view m_grid;
  /* clang-format on */
};

NS_END(block_dist, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_RANDOM_DISTRIBUTOR_HPP_ */
