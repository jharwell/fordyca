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

#include "fordyca/params/arena/block_dist_params.hpp"
#include "fordyca/representation/arena_grid.hpp"
#include "fordyca/representation/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);
namespace er = rcppsw::er;
namespace ds = rcppsw::ds;
namespace math = rcppsw::math;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
powerlaw_distributor::powerlaw_distributor(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::arena::block_dist_params* const params)
    : base_distributor(server),
      m_arena_resolution(params->arena_resolution),
      m_n_clusters(params->powerlaw.n_clusters),
      m_dist_map(),
      m_pwrdist(params->powerlaw.pwr_min, params->powerlaw.pwr_max, 2) {
  insmod("powerlaw_dist", er::er_lvl::DIAG, er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool powerlaw_distributor::distribute_block(
    std::shared_ptr<representation::base_block>& block,
    entity_list& entities) {
  /*
   * If we get here than either all clusters of the specified capacity are
   * full and/or one or more are not full but have additional entities
   * contained within their boundaries that are taking up space (i.e. caches).
   *
   * So, change cluster size and try again.
   */
  for (auto l = m_dist_map.begin(); l != m_dist_map.end(); ++l) {
    for (auto& d : l->second) {
      ER_NOM(
          "Attempting distribution of block%d to cluster: capacity=%u, "
          "count=%u",
          block->id(),
          l->first,
          d.cluster().block_count());

      if (d.distribute_block(block, entities)) {
        return true;
      }
    } /* for(&d..) */
  }   /* for(i..) */

  ER_FATAL_SENTINEL("FATAL: Unable to distribute block to any cluster");
  return false;
} /* distribute_block() */

bool powerlaw_distributor::distribute_blocks(block_vector& blocks,
                                             entity_list& entities) {
  /*
   * Try the distributors in a random order each time. If they all fail to
   * distribute the block (for whatever reason) return the error.
   */
  return std::all_of(blocks.begin(),
                     blocks.end(),
                     [&](std::shared_ptr<representation::base_block>& b) {
                       return distribute_block(b, entities);
                     });
} /* distribute_blocks() */

powerlaw_distributor::arena_view_list powerlaw_distributor::guess_cluster_placements(
    representation::arena_grid& grid,
    const std::vector<uint>& clust_sizes) {
  arena_view_list views;

  for (size_t i = 0; i < clust_sizes.size(); ++i) {
    std::uniform_int_distribution<int> xgen(
        clust_sizes[i] / 2 + 1, grid.xdsize() - clust_sizes[i] / 2 - 1);
    std::uniform_int_distribution<int> ygen(
        clust_sizes[i] / 2 + 1, grid.ydsize() - clust_sizes[i] / 2 - 1);

    uint x = xgen(m_rng);
    uint y = ygen(m_rng);
    uint x_max = x + std::sqrt(clust_sizes[i]);
    uint y_max = y + clust_sizes[i] / (x_max - x);

    auto view = grid.subgrid(x, y, x_max, y_max);
    ER_VER(
        "Guess cluster%zu placement: x=[%lu-%lu], y=[%lu-%lu], size=%u",
        i,
        (*view.origin())->loc().first + view.index_bases()[0],
        (*view.origin())->loc().first + view.index_bases()[0] + view.shape()[0],
        (*view.origin())->loc().second + view.index_bases()[1],
        (*view.origin())->loc().second + view.index_bases()[1] + view.shape()[1],
        clust_sizes[i]);
    views.push_back(std::make_pair(view, clust_sizes[i]));
  } /* for(i..) */
  return views;
} /* guess_cluster_placements() */

__rcsw_pure bool powerlaw_distributor::check_cluster_placements(
    const arena_view_list& list) {
  for (auto& v : list) {
    bool overlap = std::any_of(
        list.begin(),
        list.end(),
        [&](const std::pair<representation::arena_grid::view, uint>& other) {
          if (other == v) { /* self */
            return false;
          }
          uint v_xbase = (*v.first.origin())->loc().first;
          uint v_ybase = (*v.first.origin())->loc().second;
          uint other_xbase = (*other.first.origin())->loc().first;
          uint other_ybase = (*other.first.origin())->loc().second;
          math::range<uint> v_xrange(v_xbase + v.first.index_bases()[0],
                                     v_xbase + v.first.index_bases()[0] +
                                         v.first.shape()[0]);
          math::range<uint> v_yrange(v_ybase + v.first.index_bases()[1],
                                     v_ybase + v.first.index_bases()[1] +
                                         v.first.shape()[1]);
          math::range<uint> other_xrange(
              other_xbase + other.first.index_bases()[0],
              other_xbase + other.first.index_bases()[0] +
                  other.first.shape()[0]);
          math::range<uint> other_yrange(
              other_ybase + other.first.index_bases()[1],
              other_ybase + other.first.index_bases()[1] +
                  other.first.shape()[1]);

          return v_xrange.overlaps_with(other_xrange) &&
                 v_yrange.overlaps_with(other_yrange);
        });
    if (overlap) {
      return false;
    }
  } /* for(&v..) */
  return true;
} /* check_cluster_placements() */

powerlaw_distributor::arena_view_list powerlaw_distributor::
    compute_cluster_placements(representation::arena_grid& grid,
                               uint n_clusters) {
  ER_NOM("Computing cluster placements for %u clusters", n_clusters);

  std::vector<uint> clust_sizes;
  for (uint i = 0; i < n_clusters; ++i) {
    /* can't have a cluster of size 0 */
    uint index = std::max(1.0, m_pwrdist(m_rng));
    ER_DIAG("Cluster%u size=%d", i, index);
    clust_sizes.push_back(index);
  } /* for(i..) */

  for (size_t i = 0; i < kMAX_DIST_TRIES; ++i) {
    auto views = guess_cluster_placements(grid, clust_sizes);
    if (check_cluster_placements(views)) {
      return views;
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL(
      "FATAL: Unable to place clusters in arena (impossible situation?)");
  return arena_view_list{};
} /* compute_cluster_placements() */

bool powerlaw_distributor::map_clusters(representation::arena_grid& grid) {
  arena_view_list placements = compute_cluster_placements(grid, m_n_clusters);
  if (0 == placements.size()) {
    ER_WARN("WARNING: Unable to compute all cluster placements");
    return false;
  }

  for (auto placement : placements) {
    m_dist_map[placement.second].emplace_back(
        server_ref(), placement.first, m_arena_resolution, placement.second);
  } /* for(i..) */
  for (auto it = m_dist_map.begin(); it != m_dist_map.end(); ++it) {
    ER_NOM("Mapped %zu clusters of capacity %u", it->second.size(), it->first);
    for (auto dist : it->second) {
      ER_DIAG("Cluster with origin@(%u, %u): capacity=%u",
              (*dist.cluster().view().origin())->loc().first,
              (*dist.cluster().view().origin())->loc().second,
              dist.cluster().capacity());
    } /* for(dist..) */
  }   /* for(&l..) */
  return true;
} /* map_clusters() */

NS_END(block_dist, support, fordyca);