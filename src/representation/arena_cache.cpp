/**
 * @file arena_cache.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/representation/arena_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_cache::arena_cache(double dimension,
                         double resolution,
                         rmath::vector2d center,
                         const ds::block_vector& blocks,
                         int id)
    : base_cache(dimension, resolution, center, blocks, id) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_cache::reset_metrics(void) {
  m_block_pickups = 0;
  m_block_drops = 0;
  m_penalty_count = 0;
} /* reset_metrics() */

NS_END(fordyca, representation);
