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
constexpr char dispatcher::kDIST_RANDOM[];
constexpr char dispatcher::kDIST_SINGLE_SRC[];
constexpr char dispatcher::kDIST_POWERLAW[];

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dispatcher::dispatcher(ds::arena_grid& grid,
                       const struct params::arena::block_dist_params* const params)
    : mc_params(*params),
      m_dist_type(params->dist_type),
      m_grid(grid),
      m_dist(nullptr) {}
dispatcher::~dispatcher(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool dispatcher::initialize(void) {
  ds::arena_grid::view arena = m_grid.layer<arena_grid::kCell>()->subgrid(
      2, 3, m_grid.xdsize() - 2, m_grid.ydsize() - 2);
  if (kDIST_RANDOM == m_dist_type) {
    m_dist = rcppsw::make_unique<random_distributor>(arena,
                                                     mc_params.arena_resolution);
  } else if (kDIST_SINGLE_SRC == m_dist_type) {
    ds::arena_grid::view area = m_grid.layer<arena_grid::kCell>()->subgrid(
        m_grid.xdsize() * 0.80, 2, m_grid.xdsize() * 0.90, m_grid.ydsize() - 2);
    m_dist = rcppsw::make_unique<cluster_distributor>(
        area, mc_params.arena_resolution, std::numeric_limits<uint>::max());
  } else if (kDIST_POWERLAW == m_dist_type) {
    auto p = rcppsw::make_unique<powerlaw_distributor>(&mc_params);
    if (!p->map_clusters(m_grid)) {
      return false;
    }
    m_dist = std::move(p);
  }
  return true;
} /* initialize() */

bool dispatcher::distribute_block(
    std::shared_ptr<representation::base_block>& block,
    entity_list& entities) {
  return m_dist->distribute_block(block, entities);
} /* distribute_block() */

bool dispatcher::distribute_blocks(block_vector& blocks, entity_list& entities) {
  return m_dist->distribute_blocks(blocks, entities);
} /* distribute_block() */

NS_END(block_dist, support, fordyca);
