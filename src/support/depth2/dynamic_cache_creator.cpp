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
                                             double min_dist,
                                             uint min_blocks)
    : base_cache_creator(grid, cache_dim),
      ER_CLIENT_INIT("fordyca.support.depth2.dynamic_cache_creator"),
      m_min_dist(min_dist),
      m_min_blocks(min_blocks) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ds::cache_vector dynamic_cache_creator::create_all(
    const ds::cache_vector& existing_caches,
    ds::block_vector& candidate_blocks) {
  ds::cache_vector caches;

  std::string s =
      std::accumulate(candidate_blocks.begin(),
                      candidate_blocks.end(),
                      std::string(),
                      [&](const std::string& a, const auto& b) {
                        return a + "b" + std::to_string(b->id()) + ",";
                      });

  ER_DEBUG("Creating caches: min_dist=%f,min_blocks=%u,free blocks=[%s] (%zu)",
           m_min_dist,
           m_min_blocks,
           s.c_str(),
           candidate_blocks.size());

  block_list used_blocks;
  for (size_t i = 0; i < candidate_blocks.size() - 1; ++i) {
    block_list cache_i_blocks;

    /*
     * Block already in a new cache
     */
    if (std::find(used_blocks.begin(), used_blocks.end(), candidate_blocks[i]) !=
        used_blocks.end()) {
      continue;
    }
    for (size_t j = i + 1; j < candidate_blocks.size(); ++j) {
      /*
       * First, we have to first a block j that is close enough to block i to be
       * able to create a cache.
       */
      if ((candidate_blocks[i]->real_loc() - candidate_blocks[j]->real_loc())
              .Length() <= m_min_dist) {
        /*
         * We don't want to double add any blocks.
         */
        if (std::find(cache_i_blocks.begin(),
                      cache_i_blocks.end(),
                      candidate_blocks[i]) == cache_i_blocks.end()) {
          ER_TRACE("Add block %zu: (%f, %f)",
                   i,
                   candidate_blocks[i]->real_loc().GetX(),
                   candidate_blocks[i]->real_loc().GetY());
          cache_i_blocks.push_back(candidate_blocks[i]);
        }
        if (std::find(cache_i_blocks.begin(),
                      cache_i_blocks.end(),
                      candidate_blocks[j]) == cache_i_blocks.end()) {
          ER_TRACE("Add block %zu: (%f, %f)",
                   j,
                   candidate_blocks[j]->real_loc().GetX(),
                   candidate_blocks[j]->real_loc().GetY());
          cache_i_blocks.push_back(candidate_blocks[j]);
        }
      }
    } /* for(j..) */

    /*
     * We now have all the blocks that are close enough to block i to be
     * included in a new cache, so create the cache, if we have enough.
     */
    if (cache_i_blocks.size() >= m_min_blocks) {
      /*
       * We need to also take the caches we have already created into account
       * when selecting the location of the new cache, not just the caches that
       * currently exist.
       */
      ds::cache_vector avoid = existing_caches;
      avoid.insert(avoid.end(), caches.begin(), caches.end());
      argos::CVector2 center = calc_center(cache_i_blocks, avoid);
      auto cache_p = std::shared_ptr<representation::arena_cache>(
          create_single_cache(cache_i_blocks, center));
      caches.push_back(cache_p);

      /* Need to make sure we don't use these blocks in any other caches */
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
    const ds::cache_vector& existing_caches) const {
  double sumx = std::accumulate(
      std::begin(blocks), std::end(blocks), 0.0, [](double sum, const auto& b) {
        return sum + b->real_loc().GetX();
      });
  double sumy = std::accumulate(
      std::begin(blocks), std::end(blocks), 0.0, [](double sum, const auto& b) {
        return sum + b->real_loc().GetY();
      });

  argos::CVector2 center(sumx / blocks.size(), sumy / blocks.size());
  ER_DEBUG("Guess center=(%f,%f)", center.GetX(), center.GetY());

  /* If no existing caches, no possibility for conflict */
  if (existing_caches.empty()) {
    return center;
  }
  std::string s =
      std::accumulate(existing_caches.begin(),
                      existing_caches.end(),
                      std::string(),
                      [&](const std::string& a, const auto& b) {
                        return a + "c" + std::to_string(b->id()) + ",";
                      });

  ER_DEBUG("Deconflict caches=[%s]", s.c_str());

  /*
   * Every time we find an overlap we have to re-test all of the caches we've
   * already verified won't overlap with our new cache, because the move we
   * just made in x or y might have just caused an overlap.
   */
  uint i;
  for (i = 0; i < existing_caches.size() * kOVERLAP_SEARCH_MAX_TRIES; ++i) {
    bool conflict = false;
    for (size_t j = 0; j < existing_caches.size(); ++j) {
      if (i == j) {
        continue;
      }
      if (!deconflict_cache_center(*existing_caches[j], center)) {
        j = 0;
        conflict = true;
      }
    } /* for(j..) */
    if (!conflict) {
      break;
    }
  } /* for(i..) */

  /*
   * @todo This will definitely need to be changed later, because it may very
   * well be possible to have blocks close together that nevertheless cannot
   * create a cache because of potential overlaps.
   */
  ER_ASSERT(i < existing_caches.size() * kOVERLAP_SEARCH_MAX_TRIES,
            "Unable to find location that did not conflict with all caches");

  return center;
} /* calc_center() */

bool dynamic_cache_creator::deconflict_cache_center(
    const representation::base_cache& cache,
    argos::CVector2& center) const {
  std::uniform_real_distribution<double> xrnd(-1.0, 1.0);
  std::uniform_real_distribution<double> yrnd(-1.0, 1.0);

  auto exc_xspan = cache.xspan(cache.real_loc());
  auto exc_yspan = cache.yspan(cache.real_loc());
  auto newc_xspan = cache.xspan(center);
  auto newc_yspan = cache.yspan(center);

  /*
   * All caches are the same size, so we can just grab an [x,y] span from the
   * first of our known caches, and use that to ensure that caches are not
   * placed too close to the arena boundaries.
   */
  double x_max = grid()->xrsize() - exc_xspan.span() * 1.5;
  double x_min = exc_xspan.span() * 1.5;
  double y_max = grid()->yrsize() - exc_yspan.span() * 1.5;
  double y_min = exc_yspan.span() * 1.5;

  if (newc_xspan.overlaps_with(exc_xspan)) {
    ER_TRACE(
        "xspan=[%f,%f] overlap cache%d@(%f,%f) xspan=[%f-%f] center=(%f,%f)",
        newc_xspan.get_min(),
        newc_xspan.get_max(),
        cache.id(),
        cache.real_loc().GetX(),
        cache.real_loc().GetY(),
        exc_xspan.get_min(),
        exc_xspan.get_max(),
        center.GetX(),
        center.GetY());
    center.SetX(std::max(
        x_min,
        std::min(x_max, center.GetX() + xrnd(m_rng) * newc_xspan.span())));
    return false;
  } else if (newc_yspan.overlaps_with(exc_yspan)) {
    ER_TRACE(
        "yspan=[%f,%f] overlap cache%d@(%f,%f) yspan=[%f-%f] center=(%f,%f)",
        newc_xspan.get_min(),
        newc_xspan.get_max(),
        cache.id(),
        cache.real_loc().GetX(),
        cache.real_loc().GetY(),
        exc_yspan.get_min(),
        exc_yspan.get_max(),
        center.GetX(),
        center.GetY());
    center.SetY(std::max(
        y_min,
        std::min(y_max, center.GetY() + yrnd(m_rng) * newc_yspan.span())));
    return false;
  }
  return true;
} /* deconflict_cache_center() */

bool dynamic_cache_creator::creation_sanity_checks(
    const ds::cache_vector& caches) const {
  for (auto& c1 : caches) {
    for (auto& c2 : caches) {
      if (c1->id() == c2->id()) {
        continue;
      }
      for (auto& b : c1->blocks()) {
        ER_ASSERT(!c2->contains_block(b),
                  "Block%d contained in both cache%d and cache%d",
                  b->id(),
                  c1->id(),
                  c2->id());
        auto c1_xspan = c1->xspan(c1->real_loc());
        auto c2_xspan = c2->xspan(c2->real_loc());
        auto c1_yspan = c1->yspan(c1->real_loc());
        auto c2_yspan = c2->yspan(c2->real_loc());
        ER_ASSERT(!(c1_xspan.overlaps_with(c2_xspan) &&
                    c1_yspan.overlaps_with(c2_yspan)),
                  "Cache%d xspan=[%f-%f], yspan=[%f,%f] overlaps cache%d "
                  "xspan=[%f-%f],yspan=[%f,%f]",
                  c1->id(),
                  c1_xspan.get_min(),
                  c1_xspan.get_max(),
                  c1_yspan.get_min(),
                  c1_yspan.get_max(),
                  c2->id(),
                  c2_xspan.get_min(),
                  c2_xspan.get_max(),
                  c2_yspan.get_min(),
                  c2_yspan.get_max());
      } /* for(&b..) */
    }   /* for(&c2..) */
  }     /* for(&c1..) */
  return true;
} /* creation_sanity_checks() */

NS_END(depth2, support, fordyca);
