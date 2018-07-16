/**
 * @file random_block_distributor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_RANDOM_BLOCK_DISTRIBUTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_RANDOM_BLOCK_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <list>
#include <vector>
#include <random>

#include "rcppsw/common/common.hpp"
#include "rcppsw/math/dcoord.hpp"
#include "fordyca/representation/arena_grid.hpp"
#include "fordyca/support/base_block_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
class multicell_entity;
class arena_grid;
class cell2D;
} // namespace representation

NS_START(support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class random_block_distributor
 * @ingroup support
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
class random_block_distributor : public base_block_distributor {
 public:
  random_block_distributor(std::shared_ptr<rcppsw::er::server> server,
                           representation::arena_grid::view& grid,
                           double resolution);

  random_block_distributor& operator=(const random_block_distributor& s) = delete;
  random_block_distributor(const random_block_distributor& s)
      : base_block_distributor(s.server_ref()),
        m_resolution(s.m_resolution),
        m_rng(s.m_rng),
        m_grid(s.m_grid) {}

  bool distribute_blocks(block_vector& blocks, entity_list& entities) override;

  bool distribute_block(std::shared_ptr<representation::block>& block,
                        entity_list& entities) override;

 private:
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
   * @param coordv A (to be filled) vector of absolute and relative coordinates
   *               within the arena view if an available location can be found.
   */
  bool find_avail_coord(const entity_list& entity, std::vector<uint>& coordv);
  bool verify_block_dist(const representation::block& block,
                         const representation::cell2D* cell);

  // clang-format off
  double                              m_resolution;
  std::default_random_engine          m_rng{std::random_device{}()};
  representation::arena_grid::view    m_grid;
  // clang-format on
};

NS_END(support, fordyca);
#endif /* INCLUDE_FORDYCA_SUPPORT_RANDOM_BLOCK_DISTRIBUTOR_HPP_ */
