/**
 * @file cache_creator.cpp
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
#include "fordyca/support/cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<representation::cache> cache_creator::create_all(
    std::vector<representation::block>& blocks) {
  std::vector<representation::cache> caches;

  for (size_t i = 0; i < blocks.size() - 1; ++i) {
    for (size_t j = i + 1; j < blocks.size(); ++j) {
      if ((blocks[i].real_loc() - blocks[j].real_loc()).Length()) {
        caches.push_back(create_single(
            representation::cache::starter_pair_ref(blocks[i], blocks[j])));

      }
    } /* for(j..) */
  } /* for(i..) */
  return caches;
} /* create() */

representation::cache cache_creator::create_single(
    representation::cache::starter_pair_ref blocks) {
  argos::CVector2 center((blocks.first.real_loc().GetX() +
                          blocks.second.real_loc().GetX()) / 2,
                         (blocks.first.real_loc().GetY() +
                          blocks.second.real_loc().GetY()) / 2);

  blocks.first.move_out_of_sight();
  blocks.second.move_out_of_sight();

  return representation::cache(
      m_cache_size,
      representation::cache::starter_pair(&blocks.first, &blocks.second));
} /* create_single() */

NS_END(support, fordyca);
