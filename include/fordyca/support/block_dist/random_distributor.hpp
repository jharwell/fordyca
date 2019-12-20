/**
 * \file random_distributor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include <boost/optional.hpp>
#include <memory>

#include "fordyca/fordyca.hpp"
#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/support/block_dist/base_distributor.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "rcppsw/math/vector2.hpp"

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
 * \class random_distributor
 * \ingroup support
 *
 * \brief Distributes a set of blocks randomly within a specified 2D area, such
 * that no blocks overlap with each other or other entities already present in
 * the arena (nest, cache, etc.).
 *
 * - Blocks are assumed to be square (this is not checked).
 * - The dimension of the a block is assumed to be the same as the resolution of
 *   the arena map the blocks are being distributed into some part of (this is
 *   not checked).
 */
class random_distributor final : public rer::client<random_distributor>,
                                 public base_distributor {
 public:
  random_distributor(const ds::arena_grid::view& grid,
                     rtypes::discretize_ratio resolution,
                     rmath::rng* rng);

  random_distributor& operator=(const random_distributor& s) = delete;

  bool distribute_blocks(ds::block_vector& blocks,
                         ds::const_entity_list& entities) override;

  /**
   * \brief Distribution a single block in the arena.
   *
   * \param block The block to distribute.
   * \param entities Entities that need to be avoided during distribution.
   *
   * \note Holding \ref arena_map block, grid mutexes necessary to safely call
   * this function in multithreaded contexts (not handled internally).
   *
   * \return \c TRUE if the distribution was successful, \c FALSE otherwise.
   */
  bool distribute_block(std::shared_ptr<crepr::base_block2D>& block,
                        ds::const_entity_list& entities) override;
  ds::block_cluster_vector block_clusters(void) const override {
    return ds::block_cluster_vector();
  }

 private:
  struct coord_search_res_t {
    rmath::vector2u rel{};
    rmath::vector2u abs{};
  };
  /**
   * \brief The maxmimum # of times the distribution will be attempted before
   * giving up.
   */
  static constexpr uint kMAX_DIST_TRIES = 1000;

  /**
   * \brief Find coordinates for distribution that are outside the extent of the
   * all specified entities, while also accounting for block size.
   *
   * \param entities The entities to avoid.
   */
  boost::optional<coord_search_res_t> avail_coord_search(
      const ds::const_entity_list& entities,
      const rmath::vector2d& block_dim);
  bool verify_block_dist(const crepr::base_block2D* block,
                         const ds::const_entity_list& entities,
                         const ds::cell2D* cell) RCSW_PURE;

  /* clang-format off */
  const rtypes::discretize_ratio mc_resolution;
  const rmath::vector2u          mc_origin;
  const rmath::rangeu            mc_xspan;
  const rmath::rangeu            mc_yspan;
  ds::arena_grid::view           m_grid;
  /* clang-format on */
};

NS_END(block_dist, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_RANDOM_DISTRIBUTOR_HPP_ */
