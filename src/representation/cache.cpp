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
             double resolution,
             argos::CVector2 center,
             std::vector<block *> &blocks)
    : cell_entity(dimension, dimension, argos::CColor::BLUE),
      m_resolution(resolution),
      m_n_block_pickups(),
      m_n_block_drops(),
      m_blocks(blocks) {
  this->real_loc(center);
  this->discrete_loc(
      representation::real_to_discrete_coord(center, resolution));
  id(m_next_id++);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache::block_remove(block *block) {
  m_blocks.erase(std::find(m_blocks.begin(), m_blocks.end(), block));
} /* block_remove() */

std::unique_ptr<cache> cache::clone(void) const {
  return rcppsw::make_unique<cache>(cell_entity::xsize(),
                                    m_resolution,
                                    real_loc(),
                                    const_cast<std::vector<block *> &>(
                                        m_blocks));
} /* clone() */

__pure bool cache::operator==(const cache &other) const {
  return cell_entity::discrete_loc() == other.cell_entity::discrete_loc();
}

NS_END(fordyca, representation);
