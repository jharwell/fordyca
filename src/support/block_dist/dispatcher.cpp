/**
 * @file dispatcher.cpp
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
#include "fordyca/support/block_dist/dispatcher.hpp"
#include <limits>

#include "fordyca/support/block_dist/cluster_distributor.hpp"
#include "fordyca/support/block_dist/multi_cluster_distributor.hpp"
#include "fordyca/support/block_dist/powerlaw_distributor.hpp"
#include "fordyca/support/block_dist/random_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, block_dist);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dispatcher::dispatcher(ds::arena_grid* const grid,
                       rtypes::discretize_ratio resolution,
                       const config::arena::block_dist_config* const config,
                       double arena_padding)
    : mc_resolution(resolution),
      mc_padding(arena_padding),
      mc_config(*config),
      mc_dist_type(config->dist_type),
      m_grid(grid),
      m_dist(nullptr) {}
dispatcher::~dispatcher(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool dispatcher::initialize(void) {
  uint padding = static_cast<uint>(mc_padding / m_grid->resolution().v());

  /* clang-format off */
  ds::arena_grid::view arena = m_grid->layer<arena_grid::kCell>()->subgrid(
      kINDEX_MIN + padding,
      kINDEX_MIN + padding,
      m_grid->xdsize() - kINDEX_MIN - padding,
      m_grid->ydsize() - kINDEX_MIN - padding);
  if (kDistRandom == mc_dist_type) {
    m_dist = std::make_unique<random_distributor>(arena,
                                                     mc_resolution);
  } else if (kDistSingleSrc == mc_dist_type) {
    ds::arena_grid::view area = m_grid->layer<arena_grid::kCell>()->subgrid(
        static_cast<size_t>(m_grid->xdsize() * 0.80),
        kINDEX_MIN,
        static_cast<size_t>(m_grid->xdsize() * 0.90),
        m_grid->ydsize() - kINDEX_MIN - padding);
    m_dist = std::make_unique<cluster_distributor>(
        area,
        mc_resolution,
        std::numeric_limits<uint>::max());
  } else if (kDistDualSrc == mc_dist_type) {
    ds::arena_grid::view area_l = m_grid->layer<arena_grid::kCell>()->subgrid(
        static_cast<size_t>(m_grid->xdsize() * 0.10),
        kINDEX_MIN,
        static_cast<size_t>(m_grid->xdsize() * 0.20),
        m_grid->ydsize() - kINDEX_MIN - padding);
    ds::arena_grid::view area_r = m_grid->layer<arena_grid::kCell>()->subgrid(
        static_cast<size_t>(m_grid->xdsize() * 0.80),
        kINDEX_MIN,
        static_cast<size_t>(m_grid->xdsize() * 0.90),
        m_grid->ydsize() - kINDEX_MIN - padding);
    std::vector<ds::arena_grid::view> grids{area_l, area_r};
    m_dist = std::make_unique<multi_cluster_distributor>(
        grids,
        mc_resolution,
        std::numeric_limits<uint>::max());
  } else if (kDistQuadSrc == mc_dist_type) {
    /*
     * Quad source is a tricky distribution to use with static caches, so we
     * have to tweak the block cluster centers in tandem with the cache
     * locations to ensure that no segfaults results from cache/cache or
     * cache/cluster overlap. See #581.
     */
    ds::arena_grid::view area_l = m_grid->layer<arena_grid::kCell>()->subgrid(
        static_cast<size_t>(m_grid->xdsize() * 0.05),
        kINDEX_MIN,
        static_cast<size_t>(m_grid->xdsize() * 0.15),
        m_grid->ydsize() - kINDEX_MIN - padding);
    ds::arena_grid::view area_r = m_grid->layer<arena_grid::kCell>()->subgrid(
        static_cast<size_t>(m_grid->xdsize() * 0.82),
        kINDEX_MIN,
        static_cast<size_t>(m_grid->xdsize() * 0.92),
        m_grid->ydsize() - kINDEX_MIN - padding);
    ds::arena_grid::view area_b = m_grid->layer<arena_grid::kCell>()->subgrid(
        kINDEX_MIN,
        static_cast<size_t>(m_grid->ydsize() * 0.05),
        m_grid->xdsize() - kINDEX_MIN - padding,
        static_cast<size_t>(m_grid->ydsize() * 0.15));
    ds::arena_grid::view area_u = m_grid->layer<arena_grid::kCell>()->subgrid(
        kINDEX_MIN,
        static_cast<size_t>(m_grid->ydsize() * 0.82),
        m_grid->xdsize() - kINDEX_MIN - padding,
        static_cast<size_t>(m_grid->ydsize() * 0.92));
    std::vector<ds::arena_grid::view> grids{area_l, area_r, area_b, area_u};
    m_dist = std::make_unique<multi_cluster_distributor>(
        grids,
        mc_resolution,
        std::numeric_limits<uint>::max());
  } else if (kDistPowerlaw == mc_dist_type) {
    auto p = std::make_unique<powerlaw_distributor>(&mc_config.powerlaw,
                                                       mc_resolution);
    if (!p->map_clusters(m_grid)) {
      return false;
    }
    m_dist = std::move(p);
  }
  /* clang-format on */
  return true;
} /* initialize() */

bool dispatcher::distribute_block(std::shared_ptr<repr::base_block>& block,
                                  ds::const_entity_list& entities) {
  return m_dist->distribute_block(block, entities);
} /* distribute_block() */

bool dispatcher::distribute_blocks(ds::block_vector& blocks,
                                   ds::const_entity_list& entities) {
  return m_dist->distribute_blocks(blocks, entities);
} /* distribute_block() */

NS_END(block_dist, support, fordyca);
