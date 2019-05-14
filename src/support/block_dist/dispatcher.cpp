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
 * Global Variables
 ******************************************************************************/
constexpr char dispatcher::kDistRandom[];
constexpr char dispatcher::kDistSingleSrc[];
constexpr char dispatcher::kDistDualSrc[];
constexpr char dispatcher::kDistQuadSrc[];
constexpr char dispatcher::kDistPowerlaw[];

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dispatcher::dispatcher(ds::arena_grid* const grid,
                       const params::arena::block_dist_params* const params,
                       double arena_padding)
    : mc_padding(arena_padding),
      mc_params(*params),
      m_dist_type(params->dist_type),
      m_grid(grid),
      m_dist(nullptr) {}
dispatcher::~dispatcher(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool dispatcher::initialize(void) {
  uint padding = static_cast<uint>(mc_padding / m_grid->resolution());

  /* clang-format off */
  ds::arena_grid::view arena = m_grid->layer<arena_grid::kCell>()->subgrid(
      kINDEX_MIN + padding,
      kINDEX_MIN + padding,
      m_grid->xdsize() - kINDEX_MIN - padding,
      m_grid->ydsize() - kINDEX_MIN - padding);
  if (kDistRandom == m_dist_type) {
    m_dist = rcppsw::make_unique<random_distributor>(arena,
                                                     mc_params.arena_resolution);
  } else if (kDistSingleSrc == m_dist_type) {
    ds::arena_grid::view area = m_grid->layer<arena_grid::kCell>()->subgrid(
        static_cast<size_t>(m_grid->xdsize() * 0.80),
        kINDEX_MIN,
        static_cast<size_t>(m_grid->xdsize() * 0.90),
        m_grid->ydsize() - kINDEX_MIN - padding);
    m_dist = rcppsw::make_unique<cluster_distributor>(
        area,
        mc_params.arena_resolution,
        std::numeric_limits<uint>::max());
  } else if (kDistDualSrc == m_dist_type) {
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
    m_dist = rcppsw::make_unique<multi_cluster_distributor>(
        grids,
        mc_params.arena_resolution,
        std::numeric_limits<uint>::max());
  } else if (kDistQuadSrc == m_dist_type) {
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
    ds::arena_grid::view area_b = m_grid->layer<arena_grid::kCell>()->subgrid(
        kINDEX_MIN,
        static_cast<size_t>(m_grid->ydsize() * 0.10),
        m_grid->xdsize() - kINDEX_MIN - padding,
        static_cast<size_t>(m_grid->ydsize() * 0.20));
    ds::arena_grid::view area_u = m_grid->layer<arena_grid::kCell>()->subgrid(
        kINDEX_MIN,
        static_cast<size_t>(m_grid->ydsize() * 0.80),
        m_grid->xdsize() - kINDEX_MIN - padding,
        static_cast<size_t>(m_grid->ydsize() * 0.90));
    std::vector<ds::arena_grid::view> grids{area_l, area_r, area_b, area_u};
    m_dist = rcppsw::make_unique<multi_cluster_distributor>(
        grids,
        mc_params.arena_resolution,
        std::numeric_limits<uint>::max());
  } else if (kDistPowerlaw == m_dist_type) {
    auto p = rcppsw::make_unique<powerlaw_distributor>(&mc_params);
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
