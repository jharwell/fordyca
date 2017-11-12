/**
 * @file cache.cpp
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
#include "fordyca/representation/cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
int cache::m_next_id = 0;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache::cache(double dimension,
             argos::CVector2 center,
             std::list<block*> blocks) :
    cell_entity(dimension, dimension, argos::CColor::BLUE),
    m_n_block_pickups(),
    m_n_block_drops(),
    m_blocks(blocks) {
  real_loc(center);
  id(m_next_id++);
  }

NS_END(fordyca, representation);
