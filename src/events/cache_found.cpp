/**
 * @file cache_found.cpp
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
#include "fordyca/events/cache_found.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);
using ds::occupancy_grid;
using repr::base_cache;
namespace rswarm = rcppsw::swarm;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_found::cache_found(std::unique_ptr<repr::base_cache> cache)
    : cell_op(cache->discrete_loc()),
      ER_CLIENT_INIT("fordyca.events.cache_found"),
      m_cache(std::move(cache)) {}

cache_found::cache_found(const std::shared_ptr<repr::base_cache>& cache)
    : cell_op(cache->discrete_loc()),
      ER_CLIENT_INIT("fordyca.events.cache_found"),
      m_cache(cache) {}

/*******************************************************************************
 * DPO Foraging
 ******************************************************************************/
void cache_found::visit(ds::dpo_store& store) {
  /**
   * Remove any and all blocks from the known blocks set that exist in
   * the same space that a cache occupies (including the host cell).
   *
   * Such contradictions can arise if we previously saw some of the leftover
   * blocks when a cache is destroyed, and left the area before a new cache was
   * created near the old cache's location. When we return to the arena and find
   * a new cache there, we are tracking blocks that no longer exist in the
   * arena.
   */
  for (auto&& b : store.blocks().values_range()) {
    if (m_cache->contains_point(b.ent()->real_loc())) {
      ER_TRACE("Remove block%d hidden behind cache%d",
               b.ent()->id(),
               m_cache->id());
      store.block_remove(b.ent_obj());
    }
  } /* while(it..) */

  auto known = store.find(m_cache);
  rswarm::pheromone_density density;
  if (nullptr != known) {
    density = known->density();

    /*
       * Repeat pheromone deposits only affect caches that are already known and
       * that we are tracking accurately.
       */
    if (store.repeat_deposit()) {
      density.pheromone_add(rswarm::pheromone_density::kUNIT_QUANTITY);
    }
  } else {
    density.pheromone_set(ds::dpo_store::kNRD_MAX_PHEROMONE);
  }

  store.cache_update(ds::dp_cache_map::value_type(m_cache, density));
} /* visit() */

/*******************************************************************************
 * MDPO Foraging
 ******************************************************************************/
void cache_found::visit(ds::cell2D& cell) {
  cell.entity(m_cache);
  visit(cell.fsm());
  ER_ASSERT(cell.state_has_cache(),
            "Cell does not have cache after cache found event");
} /* visit() */

void cache_found::visit(fsm::cell2D_fsm& fsm) {
  /*
   * If there are more blocks in the cache than currently exist in the cell,
   * then other robots have dropped blocks in cache since the last time we saw
   * it. If there are fewer blocks in the cache than currently exist in the
   * cell, then other robots have picked up blocks from the cache since the last
   * time we saw it. In either case, synchronize the cell with the reality of
   * the cache we have just observed.
   *
   * It is also possible that a robot picks up 2nd to last block in a cache,
   * effectively removing it from its memory. If the cache reappears the same
   * timestep because another robot/the arena did something, then the robot will
   * think that the cache only has 1 block remaining (it has been deleted, but
   * because we are using shared ptrs, it will not actually be deallocated until
   * after the cell that refers to it is update (this class does said update)).
   *
   * As such, we need to make sure that we ALWAYS put the cell in the correct
   * state by sending it enough block drops. (see #323).
   */
  for (size_t i = fsm.block_count();
       i < std::max(base_cache::kMinBlocks, m_cache->n_blocks());
       ++i) {
    fsm.event_block_drop();
  } /* for(i..) */

  for (size_t i = fsm.block_count();
       i > std::max(base_cache::kMinBlocks, m_cache->n_blocks());
       --i) {
    fsm.event_block_pickup();
  } /* for(i..) */
} /* visit() */

void cache_found::visit(ds::dpo_semantic_map& map) {
  ds::cell2D& cell = map.access<occupancy_grid::kCell>(x(), y());
  rswarm::pheromone_density& density =
      map.access<occupancy_grid::kPheromone>(x(), y());
  if (!cell.state_is_known()) {
    map.known_cells_inc();
    ER_ASSERT(map.known_cell_count() <= map.xdsize() * map.ydsize(),
              "Known cell count (%u) >= arena dimensions (%ux%u)",
              map.known_cell_count(),
              map.xdsize(),
              map.ydsize());
  }

  /**
   * Remove any and all blocks from the known blocks list that exist in
   * the same space that a cache occupies.
   *
   * We can have blocks/caches overlapping (in terms of physical extent, not in
   * terms of cells), if we previously saw some of the leftover blocks when a
   * cache is destroyed, and left the area before a new cache could be
   * created. When we return to the arena and find a new cache there, we are
   * tracking blocks that no longer exist in our perception.
   */
  std::list<const std::shared_ptr<repr::base_block>*> rms;
  for (auto&& b : map.blocks().values_range()) {
    if (m_cache->contains_point(b.ent()->real_loc())) {
      ER_TRACE("Remove block%d hidden behind cache%d",
               b.ent()->id(),
               m_cache->id());
      rms.push_back(&b.ent_obj());
    }
  } /* for(&&b..) */

  for (auto&& b : rms) {
    events::cell_empty_visitor op((*b)->discrete_loc());
    op.visit(map.access<occupancy_grid::kCell>((*b)->discrete_loc()));
    map.block_remove(*b);
  } /* for(&&b..) */

  /*
   * If the cell is currently in a HAS_CACHE state, then that means that this
   * cell is coming back into our LOS with a block, when it contained a cache
   * the last time it was seen. Remove the cache/synchronize with reality.
   *
   * The density needs to be reset as well, as we are now tracking a different
   * kind of cell entity.
   */
  if (cell.state_has_block()) {
    map.block_remove(cell.block());
  }

  /*
   * If the ID of the cache we currently think resides in the cell and the ID of
   * the one we just found that actually resides there are not the same, we need
   * to reset the density for the cell, and start a new decay count.
   */
  if (cell.state_has_cache() && cell.cache()->id() != m_cache->id()) {
    density.reset();
  }

  if (map.pheromone_repeat_deposit()) {
    density.pheromone_add(rswarm::pheromone_density::kUNIT_QUANTITY);
  } else {
    /*p
     * Seeing a new cache on empty square or one that used to contain a block.
     */
    if (!cell.state_has_cache()) {
      density.reset();
      density.pheromone_add(rswarm::pheromone_density::kUNIT_QUANTITY);
    } else { /* Seeing a known cache again--set its relevance to the max */
      density.pheromone_set(ds::dpo_store::kNRD_MAX_PHEROMONE);
    }
  }
  map.cache_update(ds::dp_cache_map::value_type(m_cache, density));
  visit(cell);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cache_found::visit(controller::depth2::grp_mdpo_controller& c) {
  c.ndc_push();

  visit(*c.mdpo_perception()->map());

  c.ndc_pop();
} /* visit() */

NS_END(detail, events, fordyca);
