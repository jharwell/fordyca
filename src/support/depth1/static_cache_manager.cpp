/**
 * @file static_cache_manager.cpp
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
#include "fordyca/support/depth1/static_cache_manager.hpp"
#include "fordyca/ds/arena_grid.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/math/utils.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"
#include "fordyca/math/cache_respawn_probability.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_manager::static_cache_manager(
    const struct params::caches::caches_params* params,
    ds::arena_grid* const arena_grid,
    const argos::CVector2& cache_loc)
    : base_cache_manager(arena_grid),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_manager"),
      mc_cache_params(*params),
      mc_cache_loc(cache_loc) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::pair<bool, base_cache_manager::block_vector> static_cache_manager::
    calc_blocks_for_creation(block_vector& blocks) {
  /*
   * Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently placed on the cell where the cache is to be created
   *
   * are eligible for being used to re-create the static cache.
   */
  rcppsw::math::dcoord2 dcenter =
      math::rcoord_to_dcoord(mc_cache_loc, arena_grid()->resolution());
  block_vector to_use;
  for (auto& b : blocks) {
    if (-1 == b->robot_id() && b->discrete_loc() != dcenter) {
      to_use.push_back(b);
    }
    if (to_use.size() >= mc_cache_params.static_.size) {
      break;
    }
  } /* for(b..) */

  bool ret = true;
  if (to_use.size() < representation::base_cache::kMinBlocks) {
    /*
     * Cannot use std::accumulate for these, because that doesn't work with
     * C++14/gcc7 when you are accumulating into a different type (e.g. from a
     * set of blocks into an int).
     */
    uint count = 0;
    std::for_each(to_use.begin(),
                  to_use.end(),
                  [&](std::shared_ptr<representation::base_block>& b) {
                    count +=
                        (b->is_out_of_sight() || b->discrete_loc() == dcenter);
                  });

    std::string accum;
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      accum += "b" + std::to_string(b->id()) + "->fb" +
               std::to_string(b->robot_id()) + ",";
    });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      accum += "b" + std::to_string(b->id()) + "->(" +
               std::to_string(b->discrete_loc().first) + "," +
               std::to_string(b->discrete_loc().second) + "),";
    });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_ASSERT(to_use.size() - count < representation::base_cache::kMinBlocks,
              "For new cache @(%f, %f) [%u, %u]: %zu blocks SHOULD be "
              "available, but only %zu are (min=%zu)",
              mc_cache_loc.GetX(),
              mc_cache_loc.GetY(),
              dcenter.first,
              dcenter.second,
              to_use.size() - count,
              to_use.size(),
              representation::base_cache::kMinBlocks);
    ret = false;
  }
  if (to_use.size() < mc_cache_params.static_.size) {
    ER_WARN(
        "Not enough free blocks to meet min size for new cache @(%f, %f) [%u, "
        "%u] (%zu < %u)",
        mc_cache_loc.GetX(),
        mc_cache_loc.GetY(),
        dcenter.first,
        dcenter.second,
        to_use.size(),
        mc_cache_params.static_.size);
    ret = false;
  }
  return std::make_pair(ret, to_use);
} /* calc_blocks_for_creation() */

std::pair<bool, static_cache_manager::cache_vector> static_cache_manager::create_conditional(block_vector& blocks,
                                                                                             uint n_harvesters,
                                                                                             uint n_collectors) {
  math::cache_respawn_probability p(mc_cache_params.static_.respawn_scale_factor);
  if (p.calc(n_harvesters, n_collectors) >=
      static_cast<double>(std::rand()) / RAND_MAX) {
    return create(blocks);
  } else {
    return std::make_pair(false, cache_vector());
  }
} /* create_conditional() */

std::pair<bool, static_cache_manager::cache_vector> static_cache_manager::create(
    block_vector& blocks) {
  ER_DEBUG("(Re)-Creating static cache");
  ER_ASSERT(mc_cache_params.static_.size >=
                representation::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            mc_cache_params.static_.size,
            representation::base_cache::kMinBlocks);

  support::depth1::static_cache_creator creator(arena_grid(),
                                                mc_cache_loc,
                                                mc_cache_params.dimension);
  auto pair = calc_blocks_for_creation(blocks);
  cache_vector created;
  if (!pair.first) {
    ER_WARN("Unable to create static cache @(%f, %f): Not enough free blocks",
            mc_cache_loc.GetX(),
            mc_cache_loc.GetY());
    return std::make_pair(false, created);
  }
  /* no existing caches, so empty vector */
  created = creator.create_all(cache_vector(), pair.second);
  ER_ASSERT(1 == created.size(),
            "Wrong # caches after static create: %zu",
            created.size());

  /*
   * Any blocks that are under where the cache currently is (i.e. will be
   * hidden by it) need to be added to the cache so that there all blocks in the
   * arena are accessible. This is generally only an issue at the start of
   * simulation if random block distribution is used, but weird cases can arise
   * due to task abort+block drop as well, so it is best to be safe.
   */
  for (auto& b : blocks) {
    for (auto& c : created) {
      if (!c->contains_block(b) &&
          c->xspan(c->real_loc()).overlaps_with(b->xspan(b->real_loc())) &&
          c->yspan(c->real_loc()).overlaps_with(b->yspan(b->real_loc()))) {
        events::cell_empty empty(b->discrete_loc());
        arena_grid()->access<arena_grid::kCell>(b->discrete_loc()).accept(empty);
        events::free_block_drop op(
            b,
            math::rcoord_to_dcoord(c->real_loc(), arena_grid()->resolution()),
            arena_grid()->resolution());
        arena_grid()->access<arena_grid::kCell>(op.x(), op.y()).accept(op);
        c->block_add(b);
        ER_INFO("Hidden block%d added to cache%d", b->id(), c->id());
      }
    } /* for(&c..) */
  }   /* for(&b..) */

  /*
   * Must be after fixing hidden blocks, otherwise the cache host cell will
   * have a block as its entity!
   */
  creator.update_host_cells(created);
  return std::make_pair(true, created);
} /* static_cache_create() */

NS_END(depth1, support, fordyca);
