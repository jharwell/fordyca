/**
 * @file powerlaw_distributor.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_POWERLAW_DISTRIBUTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_POWERLAW_DISTRIBUTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <random>
#include <vector>
#include <map>

#include "rcppsw/er/client.hpp"
#include "fordyca/support/block_dist/cluster_distributor.hpp"
#include "rcppsw/math/binned_powerlaw_distribution.hpp"
#include "fordyca/support/block_dist/base_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class block;
class arena_grid;
} // namespace representation

namespace params { namespace arena { struct block_dist_params; }}

NS_START(support, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class powerlaw_distributor
 * @ingroup support
 *
 * @brief Distributes a block, or set of blocks, within the arena as randomly
 * placed clusters with sizes ranging [minsize, maxsize], with a power law based
 * stride of 2^x between.
 *
 * - Blocks are assumed to be the same size as arena resolution (this is not
 *   checked).
 */
class powerlaw_distributor : public base_distributor,
                             public er::client<powerlaw_distributor> {
 public:
  /**
   * @brief Initialize the distributor.
   *
   * @param server Debugging/logging server.
   */
  explicit powerlaw_distributor(const struct params::arena::block_dist_params* params);

  powerlaw_distributor(const powerlaw_distributor& s) = delete;
  powerlaw_distributor& operator=(const powerlaw_distributor& s) = delete;

  bool distribute_block(std::shared_ptr<representation::base_block>& block,
                        entity_list& entities) override;
  bool distribute_blocks(block_vector& blocks, entity_list& entities) override;

  /**
   * @brief Computer cluster locations such that no two clusters overlap, and
   * map locations and compositional block distributors into internal data
   * structures.
   *
   * @param grid The grid for the ENTIRE arena.
   *
   * @return \c TRUE iff clusters were mapped successfull, \c FALSE otherwise.
   */
  bool map_clusters(representation::arena_grid& grid);

 private:
  using arena_view_list = std::list<std::pair<representation::arena_grid::view, uint>>;
  /**
   * @brief How many times to attempt to distribute all blocks before giving up,
   * causing an assertion failure on distribution.
   */
  static constexpr uint kMAX_DIST_TRIES = 100;

  /**
   * @brief Assign cluster centers randomly, with the only restriction that the
   * edges of each cluster are within the boundaries of the arena.
   *
   * @param grid Arena grid.
   * @param clust_sizes Vector of powers of 2 for the cluster sizes.
   */
  arena_view_list guess_cluster_placements(representation::arena_grid& grid,
                                             const std::vector<uint>& clust_sizes);

  /**
   * @brief Verify that no cluster placements cause overlap, after guessing
   * initial locations.
   *
   * @param list Possible list of cluster placements.
   *
   * @return \c TRUE if the cluster distribute is valid, \c FALSE otherwise.
   */
  bool check_cluster_placements(const arena_view_list& list);

  /**
   * @brief Perform a "guess and check" cluster placement until you get a
   * distribution without overlap, or \ref kMAX_DIST_TRIES is exceeded,
   * whichever happens first.
   *
   * Cluster sizes are drawn from the internally stored power law distribution.
   */
  arena_view_list compute_cluster_placements(representation::arena_grid& grid,
                                             uint n_clusters);

  // clang-format off
  double                                         m_arena_resolution{0.0};
  uint                                           m_n_clusters{0};
  std::map<uint, std::list<cluster_distributor>> m_dist_map;
  std::default_random_engine                     m_rng {std::random_device {}()};
  rcppsw::math::binned_powerlaw_distribution     m_pwrdist;
  // clang-format on
};

NS_END(block_dist, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BLOCK_DIST_POWERLAW_DISTRIBUTOR_HPP_ */
