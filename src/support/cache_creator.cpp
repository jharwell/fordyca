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
 * Constructors/Destructor
 ******************************************************************************/
cache_creator::cache_creator(
    std::shared_ptr<rcppsw::common::er_server> server,
    std::shared_ptr<std::vector<representation::block>> blocks,
    double min_dist, double cache_size) :
    er_client(server), m_min_dist(min_dist), m_cache_size(cache_size),
    m_blocks(blocks) {
      insmod("cache_creator",
           rcppsw::common::er_lvl::DIAG,
           rcppsw::common::er_lvl::NOM);
    }


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<representation::cache> cache_creator::create_all(
    std::vector<representation::cache>& caches) {

  std::vector<representation::block*> free_blocks;

  /*
   * We only consider blocks that do not currently belong to caches.
   */
  for (auto block : *m_blocks) {
    bool free = true;
    for (auto cache : caches) {
      if (cache.contains_block(&block)) {
        free = false;
        break;
      }
    } /* for(cache..) */
    if (free) {
      free_blocks.push_back(&block);
    }
  } /* for(block..) */
  ER_NOM("Creating caches: %zu free blocks", free_blocks.size());

  for (size_t i = 0; i < free_blocks.size() - 1; ++i) {
    std::list<representation::block*> starter_blocks;
    for (size_t j = i + 1; j < free_blocks.size(); ++j) {
      if ((free_blocks[i]->real_loc() - free_blocks[j]->real_loc()).Length() <=
          m_min_dist && i != j) {
        starter_blocks.push_back(free_blocks[i]);
      }
    } /* for(j..) */
    caches.push_back(create_single(starter_blocks));
  } /* for(i..) */
  return caches;
} /* create() */

representation::cache cache_creator::create_single(
    std::list<representation::block*> blocks) {

  for (auto block : blocks) {
    block->move_out_of_sight();
  } /* for(block..) */

  argos::CVector2 center = calc_center(blocks);

  ER_NOM("Create cache at (%f, %f) with  %zu blocks",
         center.GetX(), center.GetY(), blocks.size());
  return representation::cache(m_cache_size, calc_center(blocks), blocks);
} /* create_single() */

argos::CVector2 cache_creator::calc_center(
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
