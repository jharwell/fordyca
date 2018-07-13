/**
 * @file powerlaw_block_distributor.cpp
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
#include "fordyca/support/powerlaw_block_distributor.hpp"
#include <algorithm>
#include <cmath>
#include <random>

#include "fordyca/representation/arena_grid.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/params/block_distribution_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);
namespace er = rcppsw::er;
namespace ds = rcppsw::ds;
namespace math = rcppsw::math;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
powerlaw_block_distributor::powerlaw_block_distributor(
    std::shared_ptr<rcppsw::er::server> server,
    representation::arena_grid& grid,
    const struct params::block_distribution_params* const params)
    : base_block_distributor(server),
      m_dist_map(),
      m_pwrdist(params->pwr_min, params->pwr_max, 2) {
  insmod("powerlaw_block_dist", er::er_lvl::DIAG, er::er_lvl::NOM);
  for (auto placement : compute_cluster_placements(grid, params->n_clusters)) {
    m_dist_map[placement.second].push_back(cluster_block_distributor(server,
                                                                     placement.first,
                                                                     params->arena_resolution,
                                                                     placement.second));
  } /* for(i..) */

  for (auto it = m_dist_map.begin(); it != m_dist_map.end(); ++it) {
    ER_NOM("Distributed blocks to %zu clusters of capacity %u",
           it->second.size(),
           it->first);
    for (auto dist : it->second) {
      ER_DIAG("Cluster with origin@(%zu, %zu): capacity=%u, n_blocks=%u",
              (*dist.cluster().view().origin())->loc().first,
              (*dist.cluster().view().origin())->loc().second,
              dist.cluster().capacity(),
              dist.cluster().block_count());
    } /* for(dist..) */
  } /* for(&l..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool powerlaw_block_distributor::distribute_block(
    std::shared_ptr<representation::block>& block,
    const entity_list& entities) {
  uint sample = m_pwrdist(m_rng);
  int index = std::log2(sample);

  ER_NOM("Distributing block%d to cluster: index=%d, capacity=%u",
         block->id(),
         index,
         sample);

  for (; index > 0; --index) {
    for (auto &d : m_dist_map[index]) {
      if (d.distribute_block(block, entities)) {
        return true;
      }
    } /* for(&d..) */

    /*
     * If we get here than either all clusters of the specified capacity are full
     * and/or one or more are not full but have additional entities contained
     * within their boundaries that are taking up space (i.e. caches).
     *
     * So, drop down a cluster size and try again.
     */
  } /* for(index..) */
  ER_FATAL_SENTINEL("FATAL: Unable to distribute block to any cluster");
  return false;
} /* distribute_block() */

bool powerlaw_block_distributor::distribute_blocks(block_vector& blocks,
                                                   entity_list& entities) {
  /*
   * Try the distributors in a random order each time. If they all fail to
   * distribute the block (for whatever reason) return the error.
   */
  return std::all_of(blocks.begin(),
                     blocks.end(),
                     [&](std::shared_ptr<representation::block>& b)
                     { return distribute_block(b, entities); });
} /* distribute_blocks() */

powerlaw_block_distributor::arena_view_list powerlaw_block_distributor::guess_cluster_placements(
    representation::arena_grid& grid,
    const std::vector<uint>& clust_sizes) {
  arena_view_list views;

  for (size_t i = 0; i < clust_sizes.size(); ++i) {
    std::uniform_int_distribution<int> xgen(clust_sizes[i]/2 + 1,
                                            grid.xrsize() - clust_sizes[i]/2 - 1);
    std::uniform_int_distribution<int> ygen(clust_sizes[i]/2 + 1,
                                            grid.yrsize() - clust_sizes[i]/2 - 1);

    uint x = xgen(m_rng);
    uint y = ygen(m_rng);
    views.push_back(std::make_pair(grid.subgrid(x - clust_sizes[i]/2,
                                                y - clust_sizes[i]/2,
                                                x + clust_sizes[i]/2,
                                                y + clust_sizes[i]/2),
                                   clust_sizes[i]));
  } /* for(i..) */
  return views;
} /* guess_cluster_placements() */


bool powerlaw_block_distributor::check_cluster_placements(
    const arena_view_list& list) {

  for (auto &v : list) {
    bool overlap = std::any_of(
        list.begin(),
        list.end(),
        [&](const std::pair<representation::arena_grid::view, uint>& other) {
          if (other == v) { /* self */
            return false;
          }
          math::range<uint> v_xrange(v.first.index_bases()[0],
                                     v.first.index_bases()[0] +
                                     v.first.shape()[0]);
          math::range<uint> v_yrange(v.first.index_bases()[1],
                                     v.first.index_bases()[1] +
                                     v.first.shape()[1]);
          math::range<uint> other_xrange(other.first.index_bases()[0],
                                         other.first.index_bases()[0] +
                                         other.first.shape()[0]);
          math::range<uint> other_yrange(other.first.index_bases()[1],
                                         other.first.index_bases()[1] +
                                         other.first.shape()[1]);
          return v_xrange.overlaps_with(other_xrange) ||
          v_yrange.overlaps_with(other_yrange);
        });
    if (overlap) {
      return false;
    }
  } /* for(&v..) */
  return true;
} /* check_cluster_placements() */

powerlaw_block_distributor::arena_view_list powerlaw_block_distributor::compute_cluster_placements(
    representation::arena_grid& grid,
    uint n_clusters) {
  ER_NOM("Computing cluster placements for %u clusters", n_clusters);

  std::vector<uint> clust_sizes;
  for (uint i = 0; i < n_clusters; ++i) {
    clust_sizes.push_back(m_pwrdist(m_rng));
  } /* for(i..) */

  for (size_t i = 0; i < kMAX_DIST_TRIES; ++i) {
    auto views = guess_cluster_placements(grid, clust_sizes);
    if (check_cluster_placements(views)) {
      return views;
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL("FATAL: Unable to distribute clusters in arena (impossible situation?)");
} /* compute_cluster_placements() */

NS_END(support, fordyca);
