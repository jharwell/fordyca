/**
 * @file base_cache.cpp
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
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 * Static Members
 ******************************************************************************/
int base_cache::m_next_id = 0;
constexpr size_t base_cache::kMinBlocks;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_cache::base_cache(const params& p)
    : multicell_entity(rmath::vector2d(p.dimension, p.dimension),
                       rcppsw::utils::color::kGRAY40),
      immovable_cell_entity(p.center, p.resolution),
      mc_resolution(p.resolution),
      m_blocks(p.blocks) {
  if (-1 == p.id) {
    this->id(m_next_id++);
  } else {
    this->id(p.id);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_cache::block_remove(const std::shared_ptr<base_block>& block) {
  m_blocks.erase(std::find(m_blocks.begin(), m_blocks.end(), block));
} /* block_remove() */

std::unique_ptr<base_cache> base_cache::clone(void) const {
  return rcppsw::make_unique<base_cache>(
      params{xsize(), mc_resolution, real_loc(), blocks(), id()});
} /* clone() */

NS_END(fordyca, repr);
