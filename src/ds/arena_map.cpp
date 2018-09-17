/**
 * @file arena_map.cpp
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
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/events/cell_cache_extent.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/support/block_manifest_processor.hpp"
#include "fordyca/representation/cube_block.hpp"
#include "fordyca/representation/ramp_block.hpp"
#include "fordyca/support/depth1/static_cache_creator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_map::arena_map(const struct params::arena::arena_map_params* params)
    : ER_CLIENT_INIT("fordyca.ds.arena_map"),
      mc_static_cache_params(params->static_cache),
      m_blocks(support::block_manifest_processor(&params->blocks.dist.manifest)
               .create_blocks()),
      m_caches(),
      m_grid(params->grid.resolution,
             static_cast<size_t>(params->grid.upper.GetX()),
             static_cast<size_t>(params->grid.upper.GetY())),
      m_nest(params->nest.dims, params->nest.center, params->grid.resolution),
      m_block_dispatcher(m_grid, &params->blocks.dist) {
  ER_INFO("real=(%fx%f), discrete=(%ux%u), resolution=%f",
          m_grid.xrsize(),
          m_grid.yrsize(),
          m_grid.xdsize(),
          m_grid.ydsize(),
          m_grid.resolution());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool arena_map::initialize(void) {
  return m_block_dispatcher.initialize();
} /* initialize() */

bool arena_map::has_robot(size_t i, size_t j) const {
  return m_grid.access<arena_grid::kRobotOccupancy>(i, j);
} /* has_robot() */

__rcsw_pure int arena_map::robot_on_block(const argos::CVector2& pos) const {
  /*
   * Caches hide blocks, add even though a robot may technically be standing on
   * a block, if it is also standing in a cache, that takes priority.
   */
  if (-1 != robot_on_cache(pos)) {
    ER_TRACE("Block hidden by cache%d", robot_on_cache(pos));
    return -1;
  }
  for (size_t i = 0; i < m_blocks.size(); ++i) {
    if (m_blocks[i]->contains_point(pos)) {
      return static_cast<int>(i);
    }
  } /* for(i..) */
  return -1;
} /* robot_on_block() */

__rcsw_pure int arena_map::robot_on_cache(const argos::CVector2& pos) const {
  for (size_t i = 0; i < m_caches.size(); ++i) {
    if (m_caches[i]->contains_point(pos)) {
      return static_cast<int>(i);
    }
  } /* for(i..) */
  return -1;
} /* robot_on_cache() */

bool arena_map::calc_blocks_for_static_cache(const argos::CVector2& center,
                                             block_vector& blocks) {
  /*
   * Only blocks that are not:
   *
   * - Currently carried by a robot
   * - Currently placed on the cell where the cache is to be created
   *
   * are eligible for being used to re-create the static cache.
   */
  rcppsw::math::dcoord2 dcenter =
      math::rcoord_to_dcoord(center, m_grid.resolution());
  for (auto& b : m_blocks) {
    if (-1 == b->robot_id() && b->discrete_loc() != dcenter) {
      blocks.push_back(b);
    }
    if (blocks.size() >= mc_static_cache_params.size) {
      break;
    }
  } /* for(b..) */

  bool ret = true;
  if (blocks.size() < representation::base_cache::kMinBlocks) {
    uint count = std::accumulate(
        m_blocks.begin(),
        m_blocks.end(),
        0,
        [&](uint sum, const std::shared_ptr<representation::base_block>& b) {
          return sum + (b->is_out_of_sight() || b->discrete_loc() == dcenter);
        });

    ER_ASSERT(count < representation::base_cache::kMinBlocks,
              "For new cache @(%f, %f) [%u, %u]: %zu >= %zu blocks SHOULD be "
              "available, but only %zu are",
              center.GetX(),
              center.GetY(),
              dcenter.first,
              dcenter.second,
              m_blocks.size() - count,
              representation::base_cache::kMinBlocks,
              blocks.size());
  } else if (blocks.size() < mc_static_cache_params.size) {
    ER_WARN(
        "Not enough free blocks to meet min size for new cache @(%f, %f) [%u, "
        "%u] (%zu < %u)",
        center.GetX(),
        center.GetY(),
        dcenter.first,
        dcenter.second,
        blocks.size(),
        mc_static_cache_params.size);
    ret = false;
  }
  return ret;
} /* calc_blocks_for_static_cache() */

bool arena_map::static_cache_create(void) {
  ER_DEBUG("(Re)-Creating static cache");
  ER_ASSERT(mc_static_cache_params.size >= representation::base_cache::kMinBlocks,
            "Static cache size %u < minimum %zu",
            mc_static_cache_params.size,
            representation::base_cache::kMinBlocks);

  argos::CVector2 center((m_grid.xrsize() + m_nest.real_loc().GetX()) / 2.0,
                         m_nest.real_loc().GetY());

  support::depth1::static_cache_creator creator(
      m_grid, center, mc_static_cache_params.dimension, m_grid.resolution());

  block_vector blocks;
  if (!calc_blocks_for_static_cache(center, blocks)) {
    ER_WARN("Unable to create static cache @(%f, %f): Not enough free blocks",
            center.GetX(),
            center.GetY());
    return false;
  }
  m_caches = creator.create_all(blocks);
  ER_ASSERT(1 == m_caches.size(), "Wrong # caches after static create: %zu",
            m_caches.size());

  /*
   * Any blocks that are under where the cache currently is (i.e. will be
   * hidden by it) need to be added to the cache so that there all blocks in the
   * arena are accessible. This is generally only an issue at the start of
   * simulation if random block distribution is used, but weird cases can arise
   * due to task abort+block drop as well, so it is best to be safe.
   */
  for (auto& b : m_blocks) {
    for (auto& c : m_caches) {
      if (!c->contains_block(b) &&
          c->xspan(c->real_loc()).overlaps_with(b->xspan(b->real_loc())) &&
          c->yspan(c->real_loc()).overlaps_with(b->yspan(b->real_loc()))) {
        events::cell_empty empty(b->discrete_loc());
        m_grid.access<arena_grid::kCell>(b->discrete_loc()).accept(empty);
        events::free_block_drop op(b,
                                   math::rcoord_to_dcoord(c->real_loc(),
                                                          m_grid.resolution()),
                                   m_grid.resolution());
        m_grid.access<arena_grid::kCell>(op.x(), op.y()).accept(op);
        c->block_add(b);
        ER_INFO("Hidden block%d added to cache%d", b->id(), c->id());
      }
    } /* for(&c..) */
  }   /* for(&b..) */

  /*
   * Must be after fixing hidden blocks, otherwise the cache host cell will
   * have a block as its entity!
   */
  creator.update_host_cells(m_caches);
  return true;
} /* static_cache_create() */

bool arena_map::distribute_single_block(
    std::shared_ptr<representation::base_block>& block) {
  support::block_dist::dispatcher::entity_list entities;
  for (auto& cache : m_caches) {
    entities.push_back(cache.get());
  } /* for(&cache..) */
  for (auto& b : m_blocks) {
    if (b != block) {
      entities.push_back(b.get());
    }
  } /* for(&b..) */
  entities.push_back(&m_nest);
  return m_block_dispatcher.distribute_block(block, entities);
} /* disribute_single_block() */

void arena_map::distribute_all_blocks(void) {
  // Reset all the cells to clear old references to blocks
  m_grid.reset();

  /* distribute blocks */
  support::block_dist::dispatcher::entity_list entities;
  for (auto& cache : m_caches) {
    entities.push_back(cache.get());
  } /* for(&cache..) */
  entities.push_back(&m_nest);
  bool b = m_block_dispatcher.distribute_blocks(m_blocks, entities);
  ER_ASSERT(b, "Unable to perform initial block distribution");

  /*
   * Once all blocks have been distributed, and (possibly) all caches have been
   * created via block consolidation, then all cells that do not have blocks or
   * caches are empty.
   */
  for (size_t i = 0; i < m_grid.xdsize(); ++i) {
    for (size_t j = 0; j < m_grid.ydsize(); ++j) {
      cell2D& cell = m_grid.access<arena_grid::kCell>(i, j);
      if (!cell.state_has_block() && !cell.state_has_cache()) {
        events::cell_empty op(i, j);
        cell.accept(op);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* distribute_all_blocks() */

void arena_map::cache_remove(
    const std::shared_ptr<representation::arena_cache>& victim) {
  size_t before = caches().size();
  __rcsw_unused int id = victim->id();
  m_caches.erase(std::remove(m_caches.begin(), m_caches.end(), victim));
  ER_ASSERT(caches().size() == before - 1, "cache%d not removed", id);
} /* cache_remove() */

void arena_map::cache_extent_clear(
    const std::shared_ptr<representation::arena_cache>& victim) {
  auto xspan = victim->xspan(victim->real_loc());
  auto yspan = victim->yspan(victim->real_loc());

  /*
   * To reset all cells covered by the cache's extent, we simply send them a
   * CELL_EMPTY event. EXCEPT for the cell that hosted the actual cache, because
   * it is currently in the HAS_BLOCK state as part of a \ref cached_block_pickup,
   * and clearing it here will trigger an assert later.
   */
  uint xmin = static_cast<uint>(std::ceil(xspan.get_min() / m_grid.resolution()));
  uint xmax = static_cast<uint>(std::ceil(xspan.get_max() / m_grid.resolution()));
  uint ymin = static_cast<uint>(std::ceil(yspan.get_min() / m_grid.resolution()));
  uint ymax = static_cast<uint>(std::ceil(yspan.get_max() / m_grid.resolution()));

  for (uint i = xmin; i < xmax; ++i) {
    for (uint j = ymin; j < ymax; ++j) {
      rcppsw::math::dcoord2 c = rcppsw::math::dcoord2(i, j);
      if (c != victim->discrete_loc()) {
        ER_ASSERT(victim->contains_point(
                      math::dcoord_to_rcoord(c, m_grid.resolution())),
                  "Cache%d does not contain point (%u, %u) within its extent",
                  victim->id(),
                  i,
                  j);

        auto& cell = m_grid.access<arena_grid::kCell>(i, j);
        ER_ASSERT(cell.state_in_cache_extent(),
                  "cell(%u, %u) not in CACHE_EXTENT [state=%d]",
                  i,
                  j,
                  cell.fsm().current_state());
        events::cell_empty e(c);
        cell.accept(e);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* cache_extent_clear() */

NS_END(ds, fordyca);
