/**
 * \file block_found.cpp
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
#include "fordyca/subsystem/perception/events/block_found.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/pheromone_density.hpp"

#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_found::block_found(crepr::base_block3D* block)
    : ER_CLIENT_INIT("fordyca.subsystem.perception.events.block_found"),
      cell2D_op(block->danchor2D()),
      m_block(block) {}

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void block_found::visit(cds::cell2D& cell) {
  cell.entity(m_block);
  this->visit(cell.fsm());
} /* visit() */

void block_found::visit(cfsm::cell2D_fsm& fsm) {
  if (fsm.state_has_cache()) {
    for (size_t i = fsm.block_count(); i > 1; --i) {
      fsm.event_block_pickup();
    } /* for(i..) */
  } else if (!fsm.state_has_block()) {
    fsm.event_block_drop();
  }
  ER_ASSERT(fsm.state_has_block(),
            "Perceived cell in incorrect state after block found event");
} /* visit() */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/
fsperception::model_update_result block_found::visit(fspds::dpo_store& store) {
  /*
   * If the cell in the arena that we thought contained a cache now contains a
   * block, remove the out-of-date cache.
   */
  auto caches = store.known_caches();
  auto it = caches.begin();
  while (it != caches.end()) {
    if (m_block->danchor2D() == (*it)->dcenter2D()) {
      carepr::base_cache* tmp = (*it);
      ++it;
      store.cache_remove(tmp);
    } else {
      ++it;
    }
  } /* while(it..) */

  crepr::pheromone_density density(store.pheromone_rho());
  auto* known = store.find(m_block);
  if (nullptr != known) {
    /*
     * If the block we just "found" is already known and has a different
     * location than we think it should, then our tracked version is out of date
     * and needs to be removed.
     */
    if (!known->ent()->dloccmp(*m_block)) {
      ER_INFO("Removing block%d@%s: Moved to %s",
              known->ent()->id().v(),
              known->ent()->danchor2D().to_str().c_str(),
              m_block->danchor2D().to_str().c_str());
      store.block_remove(known->ent());
      density.pheromone_set(fspds::dpo_store::kNRD_MAX_PHEROMONE);
    } else { /* block has not moved */
      density = known->density();

      /*
       * If repeat pheromone deposits are enabled, make a deposit. Otherwise,
       * just reset the pheromone density to make because we have seen the block
       * again.
       */
      if (store.repeat_deposit()) {
        density.pheromone_add(crepr::pheromone_density::kUNIT_QUANTITY);
      } else {
        density.pheromone_set(fspds::dpo_store::kNRD_MAX_PHEROMONE);
      }
    }
  } else {
    density.pheromone_set(fspds::dpo_store::kNRD_MAX_PHEROMONE);
  }

  return store.block_update(
      repr::dpo_entity<crepr::base_block3D>(m_block->clone(), density));
} /* visit() */

void block_found::visit(fspds::dpo_semantic_map& map) {
  visit(*map.store());

  auto& cell = map.access<fspds::occupancy_grid::kCell>(coord());

  if (!cell.state_is_known()) {
    map.known_cells_inc();
    ER_ASSERT(map.known_cell_count() <= map.xdsize() * map.ydsize(),
              "Known cell count (%zu) >= arena dimensions (%zux%zu)",
              map.known_cell_count(),
              map.xdsize(),
              map.ydsize());
  }
  /*
   * If the cell is currently in a HAS_CACHE state, then that means that this
   * cell is coming back into our LOS with a block, when it contained a cache
   * the last time it was seen. Remove the cache/synchronize with reality.
   *
   * The density is reset by cache_remove().
   */
  if (cell.state_has_cache()) {
    map.cache_remove(cell.cache());
  }

  auto* dpo_ent = map.store()->find(m_block);
  map.block_update(std::move(*dpo_ent));

  ER_ASSERT(cell.state_has_block(),
            "Cell@%s not in HAS_BLOCK",
            cell.loc().to_str().c_str());
  ER_ASSERT(cell.block3D()->id() == m_block->id(),
            "Block for cell@%s ID mismatch: %d/%d",
            cell.loc().to_str().c_str(),
            m_block->id().v(),
            cell.block3D()->id().v());
} /* visit() */

NS_END(events, perception, subsystem, fordyca);
