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
#include "fordyca/support/depth2/dynamic_cache_creator.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dynamic_cache_creator::dynamic_cache_creator(ds::arena_grid* const grid,
                                             double cache_dim,
                                             double min_dist)
    : base_cache_creator(grid, cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_creator"),
      m_min_dist(min_dist) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool dynamic_cache_creator::creation_sanity_checks(
    const cache_vector& caches) const {
  for (auto &c1 : caches) {
    for (auto &c2 : caches) {
      for (auto &b : c1->blocks()) {
        ER_ASSERT(!c2->contains_block(b),
                  "Block%d contained in both cache%d and cache%d",
                  b->id(),
                  c1->id(),
                  c2->id());
        auto c1_xspan = c1->xspan(c1->real_loc());
        auto c2_xspan = c2->xspan(c2->real_loc());
        auto c1_yspan = c1->yspan(c1->real_loc());
        auto c2_yspan = c2->yspan(c2->real_loc());
        ER_ASSERT(!c1_xspan.overlaps_with(c2_xspan),
                  "Cache%d xspan=[%f-%f] overlaps cache%d xspan=[%f-%f]",
                  c1->id(),
                  c1_xspan.get_min(),
                  c1_xspan.get_max(),
                  c2->id(),
                  c2_xspan.get_min(),
                  c2_xspan.get_max());
        ER_ASSERT(!c1_yspan.overlaps_with(c2_yspan),
                  "Cache%d yspan=[%f-%f] overlaps cache%d yspan=[%f-%f]",
                  c1->id(),
                  c1_yspan.get_min(),
                  c1_yspan.get_max(),
                  c2->id(),
                  c2_yspan.get_min(),
                  c2_yspan.get_max());
      } /* for(&b..) */
    } /* for(&c2..) */
  } /* for(&c1..) */
  return true;
} /* creation_sanity_checks() */

base_cache_creator::cache_vector dynamic_cache_creator::create_all(
    const cache_vector& existing_caches,
    block_vector& candidate_blocks) {
  cache_vector caches;

  std::string s =
      std::accumulate(candidate_blocks.begin(),
                      candidate_blocks.end(),
                      std::string(),
                      [&](const std::string& a,
                          const std::shared_ptr<representation::base_block>& b) {
                        return a + "b" + std::to_string(b->id()) + ",";
                      });

  ER_INFO("Dynamically creating caches from %zu free blocks [%s]",
          candidate_blocks.size(),
          s.c_str());

  block_list used_blocks;
  for (size_t i = 0; i < candidate_blocks.size() - 1; ++i) {
    block_list cache_i_blocks;

    /*
     * Block already in a new cache
     */
    if (std::find(used_blocks.begin(),
                  used_blocks.end(),
                  candidate_blocks[i]) != used_blocks.end()) {
      continue;
    }
    for (size_t j = i + 1; j < candidate_blocks.size(); ++j) {
      /*
       * First, we have to first a block j that is close enough to block i to be
       * able to create a cache.
       */
      if ((candidate_blocks[i]->real_loc() -
           candidate_blocks[j]->real_loc()).Length() <= m_min_dist) {
        /*
         * We don't want to double add any blocks.
         */
        if (std::find(cache_i_blocks.begin(),
                      cache_i_blocks.end(),
                      candidate_blocks[i]) == cache_i_blocks.end()) {
          ER_DEBUG("Add block %zu: (%f, %f)",
                   i,
                   candidate_blocks[i]->real_loc().GetX(),
                   candidate_blocks[i]->real_loc().GetY());
          cache_i_blocks.push_back(candidate_blocks[i]);
        }
        if (std::find(cache_i_blocks.begin(),
                      cache_i_blocks.end(),
                      candidate_blocks[j]) == cache_i_blocks.end()) {
          ER_DEBUG("Add block %zu: (%f, %f)",
                   i,
                   candidate_blocks[i]->real_loc().GetX(),
                   candidate_blocks[i]->real_loc().GetY());
          cache_i_blocks.push_back(candidate_blocks[j]);
        }
      }
    } /* for(j..) */

    /*
     * We now have all the blocks that are close enough to block i to be
     * included in a new cache, so create the cache.
     */
    if (!cache_i_blocks.empty()) {
      argos::CVector2 center = calc_center(cache_i_blocks, existing_caches);
      auto cache_p = std::shared_ptr<representation::arena_cache>(
          create_single_cache(cache_i_blocks, center));
      caches.push_back(cache_p);

      /*
       * Need to make sure we don't use these blocks in any other caches.
       */
      used_blocks.insert(used_blocks.end(),
                         cache_i_blocks.begin(),
                         cache_i_blocks.end());
    }
  } /* for(i..) */

  ER_ASSERT(creation_sanity_checks(caches),
            "One or more bad caches on creation");
  return caches;
} /* create() */

argos::CVector2 dynamic_cache_creator::calc_center(
    const block_list& blocks,
    const cache_vector& existing_caches) const {
  double sumx = std::accumulate(std::begin(blocks),
                                std::end(blocks),
                                0,
                                [](double sum,
                                   const auto& b) {
                                  return sum + b->real_loc().GetX();
                                });
  double sumy = std::accumulate(std::begin(blocks),
                                std::end(blocks),
                                0,
                                [](double sum,
                                   const auto& b) {
                                  return sum + b->real_loc().GetY();
                                });

  argos::CVector2 center(sumx / blocks.size(), sumy / blocks.size());
  ER_DEBUG("Guess center=(%f,%f)", center.GetX(), center.GetY());

  /*
   * Every time we find an overlap we have to re-test all of the caches we've
   * already verified won't overlap with our new cache, because the move we
   * just made in x or y might have just caused an overlap.
   */
  uint i;
  for (i = 0; i < existing_caches.size() * kOVERLAP_SEARCH_MAX_TRIES; ++i) {
    for (size_t j = 0; j < existing_caches.size(); ++j) {
      auto new_xspan = existing_caches[j]->xspan(center);
      auto new_yspan = existing_caches[j]->yspan(center);
      auto c_xspan = existing_caches[j]->xspan(existing_caches[j]->real_loc());
      auto c_yspan = existing_caches[j]->yspan(existing_caches[j]->real_loc());
      if (new_xspan.overlaps_with(c_xspan)) {
        center.SetX(center.GetX() + kOVERLAP_SEARCH_DELTA);
        ER_TRACE("Overlap X with cache%d@(%f,%f) xspan=[%f-%f] center=(%f,%f)",
                 existing_caches[j]->id(),
                 existing_caches[j]->real_loc().GetX(),
                 existing_caches[j]->real_loc().GetY(),
                 c_xspan.get_min(),
                 c_xspan.get_max(),
                 center.GetX(),
                 center.GetY());
        j = 0;
      } else if (new_yspan.overlaps_with(c_yspan)) {
        ER_TRACE("Overlap Y with cache%d@(%f,%f) yspan=[%f-%f] center=(%f,%f)",
                 existing_caches[j]->id(),
                 existing_caches[j]->real_loc().GetX(),
                 existing_caches[j]->real_loc().GetY(),
                 c_yspan.get_min(),
                 c_yspan.get_max(),
                 center.GetX(),
                 center.GetY());
        j = 0;
        center.SetY(center.GetY() + kOVERLAP_SEARCH_DELTA);
      } else {
        break;
      }
    } /* for(j..) */
  } /* for(i..) */

  /*
   * @todo This will definitely need to be changed later, because it may very
   * well be possible to have blocks close together that nevertheless cannot
   * create a cache because of potential overlaps.
   */
  ER_ASSERT(i < kOVERLAP_SEARCH_MAX_TRIES,
            "Unable to find location that did not conflict with all caches");

  return center;
} /* calc_center() */

NS_END(depth2, support, fordyca);
