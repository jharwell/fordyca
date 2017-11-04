/**
 * @file dynamic_cache_creator.cpp
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
#include "fordyca/support/dynamic_cache_creator.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_creator::dynamic_cache_creator(
    std::shared_ptr<rcppsw::common::er_server> server,
    representation::occupancy_grid& grid,
    double cache_size,
    double resolution,
    double min_dist) :
    cache_creator(server, grid, cache_size, resolution),
    m_min_dist(min_dist) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<representation::cache> dynamic_cache_creator::create_all(
    std::vector<representation::block>& blocks) {
  std::vector<representation::cache> caches;

  ER_NOM("Dynamically creating caches: %zu free blocks", blocks.size());

  for (size_t i = 0; i < blocks.size() - 1; ++i) {
    std::list<representation::block*> starter_blocks;
    for (size_t j = i + 1; j < blocks.size(); ++j) {
      if ((blocks[i].real_loc() - blocks[j].real_loc()).Length() <=
          m_min_dist) {
        if (std::find(starter_blocks.begin(),
                      starter_blocks.end(),
                      &blocks[i]) == starter_blocks.end()) {
          ER_DIAG("Add block %zu: (%f, %f)", i,blocks[i].real_loc().GetX(),
                  blocks[i].real_loc().GetY());
          starter_blocks.push_back(&blocks[i]);
        }
        starter_blocks.push_back(&blocks[j]);
        ER_DIAG("Add block %zu: (%f, %f)", j,blocks[j].real_loc().GetX(),
                blocks[j].real_loc().GetY());
      }
    } /* for(j..) */
    if (starter_blocks.size()) {
      argos::CVector2 center = calc_center(starter_blocks);
      caches.push_back(cache_creator::create_single(starter_blocks, center));
    }
  } /* for(i..) */
  return caches;
} /* create() */

argos::CVector2 dynamic_cache_creator::calc_center(
    std::list<representation::block*> blocks) {
  double x = 0;
  double y = 0;
  for (auto block : blocks) {
    x += block->real_loc().GetX();
    y += block->real_loc().GetY();
  } /* for(block..) */
  return argos::CVector2(x / blocks.size(), y / blocks.size());
} /* calc_center() */

NS_END(support, fordyca);
