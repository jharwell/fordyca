/**
 * @file powerlaw_distributor.cpp
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
#include "fordyca/support/block_dist/powerlaw_distributor.hpp"
#include <algorithm>
#include <cmath>
#include <random>

#include "fordyca/config/arena/block_dist_config.hpp"
#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);
using fordyca::ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
powerlaw_distributor::powerlaw_distributor(
    const config::arena::powerlaw_dist_config* const config,
    double arena_resolution)
    : ER_CLIENT_INIT("fordyca.support.block_dist.powerlaw"),
      mc_arena_resolution(arena_resolution),
      m_n_clusters(config->n_clusters),
      m_pwrdist(config->pwr_min, config->pwr_max, 2) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool powerlaw_distributor::distribute_block(
    std::shared_ptr<repr::base_block>& block,
    ds::const_entity_list& entities) {
  /*
   * If we get here than either all clusters of the specified capacity are
   * full and/or one or more are not full but have additional entities
   * contained within their boundaries that are taking up space (i.e. caches).
   *
   * So, change cluster size and try again.
   */
  for (auto& l : m_dist_map) {
    for (auto& dist : l.second) {
      ER_INFO(
          "Attempting distribution: block%d -> cluster [capacity=%u,count=%zu]",
          block->id(),
          l.first,
          dist.block_clusters().front()->block_count());

      if (dist.distribute_block(block, entities)) {
        return true;
      }
    } /* for(&dist..) */
  }   /* for(l..) */

  ER_FATAL_SENTINEL("Unable to distribute block to any cluster");
  return false;
} /* distribute_block() */

powerlaw_distributor::cluster_paramvec powerlaw_distributor::guess_cluster_placements(
    ds::arena_grid* const grid,
    const std::vector<uint>& clust_sizes) {
  cluster_paramvec config;

  for (size_t i = 0; i < clust_sizes.size(); ++i) {
    std::uniform_int_distribution<int> xgen(
        clust_sizes[i] / 2 + 1, grid->xdsize() - clust_sizes[i] / 2 - 1);
    std::uniform_int_distribution<int> ygen(
        clust_sizes[i] / 2 + 1, grid->ydsize() - clust_sizes[i] / 2 - 1);

    uint x = xgen(rng());
    uint y = ygen(rng());
    uint x_max = x + static_cast<uint>(std::sqrt(clust_sizes[i]));
    uint y_max = y + clust_sizes[i] / (x_max - x);

    /*
     * Cast needed so that clusters can correctly be constructed later, and they
     * DO need to be able to modify their grid view. Here we do not need it, so
     * we have to cast.
     */
    auto view = grid->layer<arena_grid::kCell>()->subgrid(x, y, x_max, y_max);
    __rcsw_unused rmath::vector2u loc = (*view.origin()).loc();
    ER_TRACE("Guess cluster%zu placement x=[%lu-%lu], y=[%lu-%lu], size=%u",
             i,
             loc.x() + view.index_bases()[0],
             loc.x() + view.index_bases()[0] + view.shape()[0],
             loc.y() + view.index_bases()[1],
             loc.y() + view.index_bases()[1] + view.shape()[1],
             clust_sizes[i]);
    config.push_back({view, clust_sizes[i]});
  } /* for(i..) */
  return config;
} /* guess_cluster_placements() */

__rcsw_pure bool powerlaw_distributor::check_cluster_placements(
    const cluster_paramvec& pvec) {
  for (const cluster_config& p : pvec) {
    bool overlap = std::any_of(pvec.begin(), pvec.end(), [&](const auto& other) {
      /*
         * Can't compare directly (boost multi_array makes a COPY of each
         * element during iteration for some reason, and because the cells
         * have a unique_ptr, that doesn't work), so compare using addresses
         * of elements of the vector, which WILL work.
         */
      if (&other == &p) { /* self */
        return false;
      }
      rmath::vector2u v_loc = (*p.view.origin()).loc();
      rmath::vector2u other_loc = (*other.view.origin()).loc();
      rmath::rangeu v_xrange(v_loc.x() + p.view.index_bases()[0],
                             v_loc.x() + p.view.index_bases()[0] +
                                 p.view.shape()[0]);
      rmath::rangeu v_yrange(v_loc.y() + p.view.index_bases()[1],
                             v_loc.y() + p.view.index_bases()[1] +
                                 p.view.shape()[1]);
      rmath::rangeu other_xrange(other_loc.x() + other.view.index_bases()[0],
                                 other_loc.x() + other.view.index_bases()[0] +
                                     other.view.shape()[0]);
      rmath::rangeu other_yrange(other_loc.y() + other.view.index_bases()[1],
                                 other_loc.y() + other.view.index_bases()[1] +
                                     other.view.shape()[1]);

      return v_xrange.overlaps_with(other_xrange) &&
             v_yrange.overlaps_with(other_yrange);
    });
    if (overlap) {
      return false;
    }
  } /* for(&p..) */
  return true;
} /* check_cluster_placements() */

powerlaw_distributor::cluster_paramvec powerlaw_distributor::
    compute_cluster_placements(ds::arena_grid* const grid, uint n_clusters) {
  ER_INFO("Computing cluster placements for %u clusters", n_clusters);

  std::vector<uint> clust_sizes;
  for (uint i = 0; i < n_clusters; ++i) {
    /* can't have a cluster of size 0 */
    uint index = static_cast<uint>(std::max(1.0, m_pwrdist(rng())));
    ER_DEBUG("Cluster%u size=%d", i, index);
    clust_sizes.push_back(index);
  } /* for(i..) */

  for (uint i = 0; i < kMAX_DIST_TRIES; ++i) {
    auto views = guess_cluster_placements(grid, clust_sizes);
    if (check_cluster_placements(views)) {
      return views;
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL(
      "Unable to place clusters in arena (impossible situation?)");
  return cluster_paramvec{};
} /* compute_cluster_placements() */

bool powerlaw_distributor::map_clusters(ds::arena_grid* const grid) {
  cluster_paramvec config = compute_cluster_placements(grid, m_n_clusters);
  if (config.empty()) {
    ER_WARN("Unable to compute all cluster placements");
    return false;
  }

  for (auto& bclustp : config) {
    m_dist_map[bclustp.capacity].emplace_back(bclustp.view,
                                              bclustp.capacity,
                                              mc_arena_resolution);
  } /* for(i..) */
  for (auto& [clust_size, dist_list] : m_dist_map) {
    ER_INFO("Mapped %zu clusters of capacity %u", dist_list.size(), clust_size);
    for (__rcsw_unused auto& dist : dist_list) {
      ER_INFO("Cluster with origin@%s: capacity=%u",
              dist.block_clusters().front()->anchor().to_str().c_str(),
              dist.block_clusters().front()->capacity());
    } /* for(dist..) */
  }   /* for(&l..) */
  return true;
} /* map_clusters() */

ds::block_cluster_vector powerlaw_distributor::block_clusters(void) const {
  ds::block_cluster_vector ret;

  for (auto& l : m_dist_map) {
    for (auto& dist : l.second) {
      auto bclusts = dist.block_clusters();
      ret.insert(ret.end(), bclusts.begin(), bclusts.end());
    } /* for(&d..) */
  }   /* for(i..) */

  return ret;
} /* block_clusters() */

NS_END(block_dist, support, fordyca);
