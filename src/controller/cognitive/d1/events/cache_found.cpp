/**
 * \file cache_found.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/cognitive/d1/events/cache_found.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cell2D_empty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

using carepr::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_found::cache_found(carepr::base_cache* cache)
    : cell2D_op(cache->dcenter2D()),
      ER_CLIENT_INIT("fordyca.controller.cognitive.d1.events.cache_found"),
      m_cache(cache) {}

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cache_found::visit(fccd1::bitd_mdpo_controller& c) {
  c.ndc_uuid_push();

  visit(*c.perception()->model<fspds::dpo_semantic_map>());

  c.ndc_uuid_pop();
} /* visit() */

void cache_found::visit(fccd1::bitd_dpo_controller& c) {
  c.ndc_uuid_push();

  visit(*c.perception()->model<fspds::dpo_store>());

  c.ndc_uuid_pop();
} /* visit() */

void cache_found::visit(fccd1::bitd_omdpo_controller& c) {
  c.ndc_uuid_push();

  visit(*c.perception()->model<fspds::dpo_semantic_map>());

  c.ndc_uuid_pop();
} /* visit() */

void cache_found::visit(fccd1::bitd_odpo_controller& c) {
  c.ndc_uuid_push();

  visit(*c.perception()->model<fspds::dpo_store>());

  c.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void cache_found::visit(cfsm::cell2D_fsm& fsm) {
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
   * state by sending it enough block drops. (see FORDYCA#323).
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

/*******************************************************************************
 * Data Structures
 ******************************************************************************/
void cache_found::visit(fspds::dpo_store& store) {
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
  auto blocks = store.known_blocks();
  auto it = blocks.begin();
  while (it != blocks.end()) {
    if (m_cache->contains_point((*it)->rcenter2D())) {
      crepr::sim_block3D* tmp = (*it);
      ++it;
      ER_TRACE("Remove block%d hidden behind cache%d",
               tmp->id().v(),
               m_cache->id().v());
      store.block_remove(tmp);
    } else {
      ++it;
    }
  } /* while(it..) */

  auto* known = store.find(m_cache);
  crepr::pheromone_density density(store.pheromone_rho());
  if (nullptr != known) {
    density = known->density();

    /*
     * If repeat pheromon deposits are enabled, make a deposit. Otherwise, just
     * reset the pheromone density to make because we have seen the cache again.
     */
    if (store.repeat_deposit()) {
      density.pheromone_add(crepr::pheromone_density::kUNIT_QUANTITY);
    } else {
      density.pheromone_set(fspds::dpo_store::kNRD_MAX_PHEROMONE);
    }
  } else {
    density.pheromone_set(fspds::dpo_store::kNRD_MAX_PHEROMONE);
  }

  auto ent = fspds::dp_cache_map::value_type(m_cache->clone(), density);
  store.cache_update(std::move(ent));
} /* visit() */

/*******************************************************************************
 * MDPO Foraging
 ******************************************************************************/
void cache_found::visit(cds::cell2D& cell) {
  cell.entity(m_cache);
  visit(cell.fsm());
  ER_ASSERT(cell.state_has_cache(),
            "Cell does not have cache after cache found event");
} /* visit() */

void cache_found::visit(fspds::dpo_semantic_map& map) {
  cds::cell2D& cell = map.access<fspds::occupancy_grid::kCell>(x(), y());
  crepr::pheromone_density& density =
      map.access<fspds::occupancy_grid::kPheromone>(x(), y());
  if (!cell.state_is_known()) {
    map.known_cells_inc();
    ER_ASSERT(map.known_cell_count() <= map.xdsize() * map.ydsize(),
              "Known cell count (%zu) >= arena dimensions (%zux%zu)",
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
  std::vector<crepr::sim_block3D*> rms;
  auto blocks = map.known_blocks();
  for (auto&& b : blocks) {
    if (m_cache->contains_point(b->rcenter2D())) {
      ER_TRACE("Remove block%d hidden behind cache%d",
               b->id().v(),
               m_cache->id().v());
      rms.push_back(b);
    }
  } /* for(&&b..) */

  for (auto&& b : rms) {
    cdops::cell2D_empty_visitor op(b->danchor2D());
    op.visit(map.access<fspds::occupancy_grid::kCell>(b->danchor2D()));
    map.block_remove(b);
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
    map.block_remove(cell.block3D());
  }

  /*
   * The cache we get a handle to is owned by the simulation, and we don't
   * want to just pass that into the robot's arena_map, as keeping them in
   * sync is not possible in all situations.
   *
   * For example, if a block executing the collector task picks up a block
   * and tries to compute the best cache to bring it to, only to have one or
   * more of its cache references be invalid due to other robots causing
   * caches to be created/destroyed.
   *
   * Cloning is definitely necessary here.
   */
  map.cache_update({m_cache->clone(), density});
} /* visit() */

NS_END(events, d1, cognitive, controller, fordyca);
