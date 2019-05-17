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
#include "fordyca/math/cache_respawn_probability.hpp"
#include "fordyca/repr/arena_cache.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
static_cache_manager::static_cache_manager(
    const config::caches::caches_config* config,
    ds::arena_grid* const arena_grid,
    const rmath::vector2d& cache_loc)
    : base_cache_manager(arena_grid),
      ER_CLIENT_INIT("fordyca.support.depth1.static_cache_manager"),
      mc_cache_config(*config),
      mc_cache_loc(cache_loc),
      m_reng(std::chrono::system_clock::now().time_since_epoch().count()) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<ds::block_vector> static_cache_manager::calc_blocks_for_creation(
    ds::block_vector& blocks) {
  /*
   * Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently placed on the cell where the cache is to be created
   *
   * are eligible for being used to re-create the static cache.
   */
  rmath::vector2u dcenter =
      rmath::dvec2uvec(mc_cache_loc, arena_grid()->resolution());
  ds::block_vector to_use;
  for (auto& b : blocks) {
    if (-1 == b->robot_id() && b->discrete_loc() != dcenter) {
      to_use.push_back(b);
    }
    if (to_use.size() >= mc_cache_config.static_.size) {
      break;
    }
  } /* for(b..) */

  if (to_use.size() < repr::base_cache::kMinBlocks) {
    /*
     * Cannot use std::accumulate for these, because that doesn't work with
     * C++14/gcc7 when you are accumulating into a different type (e.g. from a
     * set of blocks into an int).
     */
    uint count = 0;
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      count += (b->is_out_of_sight() || b->discrete_loc() == dcenter);
    });

    std::string accum;
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      accum += "b" + std::to_string(b->id()) + "->fb" +
               std::to_string(b->robot_id()) + ",";
    });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(to_use.begin(), to_use.end(), [&](const auto& b) {
      accum += "b" + std::to_string(b->id()) + "->" +
               b->discrete_loc().to_str() + ",";
    });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_ASSERT(to_use.size() - count < repr::base_cache::kMinBlocks,
              "For new cache @%s/%s: %zu blocks SHOULD be "
              "available, but only %zu are (min=%zu)",
              mc_cache_loc.to_str().c_str(),
              dcenter.to_str().c_str(),
              to_use.size() - count,
              to_use.size(),
              repr::base_cache::kMinBlocks);
    return boost::optional<ds::block_vector>();
  }
  if (to_use.size() < mc_cache_config.static_.size) {
    ER_WARN(
        "Not enough free blocks to meet min size for new cache@%s/%s (%zu < "
        "%u)",
        mc_cache_loc.to_str().c_str(),
        dcenter.to_str().c_str(),
        to_use.size(),
        mc_cache_config.static_.size);
    return boost::optional<ds::block_vector>();
  }
  return boost::make_optional(to_use);
} /* calc_blocks_for_creation() */

boost::optional<ds::cache_vector> static_cache_manager::create_conditional(
    ds::block_vector& blocks,
    uint timestep,
    uint n_harvesters,
    uint n_collectors) {
  math::cache_respawn_probability p(
      mc_cache_config.static_.respawn_scale_factor);
  std::uniform_real_distribution<> dist(0.0, 1.0);
  if (p.calc(n_harvesters, n_collectors) >= dist(m_reng)) {
    return create(blocks, timestep);
  } else {
    return boost::optional<ds::cache_vector>();
  }
} /* create_conditional() */

boost::optional<ds::cache_vector> static_cache_manager::create(
    ds::block_vector& blocks,
    uint timestep) {
  ER_DEBUG("(Re)-Creating static cache");
  ER_ASSERT(mc_cache_config.static_.size >= repr::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            mc_cache_config.static_.size,
            repr::base_cache::kMinBlocks);

  auto to_use = calc_blocks_for_creation(blocks);
  if (!to_use) {
    ER_WARN("Unable to create static cache @%s: Not enough free blocks",
            mc_cache_loc.to_str().c_str());
    return boost::optional<ds::cache_vector>();
  }
  ds::cache_vector created;

  support::depth1::static_cache_creator creator(arena_grid(),
                                                mc_cache_loc,
                                                mc_cache_config.dimension);

  /* no existing caches, so empty vector */
  created = creator.create_all(ds::cache_vector(), *to_use, timestep);
  ER_ASSERT(1 == created.size(),
            "Wrong # caches after static create: %zu",
            created.size());
  caches_created(1);

  /*
   * Fix hidden blocks and update host cells. Host cell updating must be second,
   * otherwise the cache host cell will have a block as its entity!
   */
  post_creation_blocks_adjust(created, blocks);
  creator.update_host_cells(created);
  return boost::make_optional(created);
} /* create() */

void static_cache_manager::post_creation_blocks_adjust(
    const ds::cache_vector& caches,
    const ds::block_vector& blocks) {
  for (auto& b : blocks) {
    for (auto& c : caches) {
      if (!c->contains_block(b) &&
          c->xspan(c->real_loc()).overlaps_with(b->xspan(b->real_loc())) &&
          c->yspan(c->real_loc()).overlaps_with(b->yspan(b->real_loc()))) {
        events::cell_empty_visitor empty(b->discrete_loc());
        empty.visit(arena_grid()->access<arena_grid::kCell>(b->discrete_loc()));
        events::free_block_drop_visitor op(
            b,
            rmath::dvec2uvec(c->real_loc(), arena_grid()->resolution()),
            arena_grid()->resolution());
        op.visit(arena_grid()->access<arena_grid::kCell>(op.x(), op.y()));
        c->block_add(b);
        ER_INFO("Hidden block%d added to cache%d", b->id(), c->id());
      }
    } /* for(&c..) */
  }   /* for(&b..) */
} /* post_creation_blocks_adjust() */
NS_END(depth1, support, fordyca);
