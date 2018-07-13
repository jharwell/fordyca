/**
 * @file block_distribution_dispatcher.cpp
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
#include "fordyca/support/block_distribution_dispatcher.hpp"
#include <limits>

#include "fordyca/params/block_distribution_params.hpp"
#include "fordyca/support/random_block_distributor.hpp"
#include "fordyca/support/cluster_block_distributor.hpp"
#include "fordyca/support/powerlaw_block_distributor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_distribution_dispatcher::kDIST_RANDOM[];
constexpr char block_distribution_dispatcher::kDIST_SINGLE_SRC[];
constexpr char block_distribution_dispatcher::kDIST_POWERLAW[];

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_distribution_dispatcher::block_distribution_dispatcher(
    std::shared_ptr<rcppsw::er::server> server,
    representation::arena_grid& grid,
    const struct params::block_distribution_params* const params)
    : client(server),
      m_dist_type(params->dist_type),
      m_grid(grid),
      m_dist(nullptr) {
  initialize(params);
}
block_distribution_dispatcher::~block_distribution_dispatcher(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_distribution_dispatcher::initialize(
    const struct params::block_distribution_params* params) {
  if (kDIST_RANDOM == m_dist_type) {
    representation::arena_grid::view arena = m_grid.subgrid(1,
                                                            1,
                                                            m_grid.xrsize() - 1,
                                                            m_grid.yrsize() - 1);

    m_dist = rcppsw::make_unique<random_block_distributor>(client::server_ref(),
                                                           arena,
                                                           params->arena_resolution);
  } else if (kDIST_SINGLE_SRC == m_dist_type) {
    representation::arena_grid::view area = m_grid.subgrid(m_grid.xrsize() * 0.85,
                                                           m_grid.yrsize() * 0.05,
                                                           m_grid.xrsize() * 0.90,
                                                           m_grid.yrsize() * 0.95);
    m_dist = rcppsw::make_unique<cluster_block_distributor>(client::server_ref(),
                                                            area,
                                                            params->arena_resolution,
                                                            std::numeric_limits<uint>::max());
  } else if (kDIST_POWERLAW == m_dist_type) {
    m_dist = rcppsw::make_unique<powerlaw_block_distributor>(client::server_ref(),
                                                             m_grid,
                                                             params);
  }
} /* initialize() */

bool block_distribution_dispatcher::distribute_block(
    std::shared_ptr<representation::block>& block,
    const entity_list& entities) {
  return m_dist->distribute_block(block, entities);
} /* distribute_block() */

bool block_distribution_dispatcher::distribute_blocks(block_vector& blocks,
                                                      entity_list& entities) {
  return m_dist->distribute_blocks(blocks, entities);
} /* distribute_block() */

NS_END(support, fordyca);
