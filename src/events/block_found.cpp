/**
 * @file block_found.cpp
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
#include "fordyca/events/block_found.hpp"
#include "fordyca/controller/depth1/gp_dpo_controller.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth1/gp_odpo_controller.hpp"
#include "fordyca/controller/depth1/gp_omdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_odpo_controller.hpp"
#include "fordyca/controller/depth2/grp_omdpo_controller.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/repr/base_cache.hpp"
#include "rcppsw/swarm/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_found::block_found(std::unique_ptr<repr::base_block> block)
    : ER_CLIENT_INIT("fordyca.events.block_found"),
      cell_op(block->dloc()),
      m_block(std::move(block)) {}

block_found::block_found(const std::shared_ptr<repr::base_block>& block)
    : ER_CLIENT_INIT("fordyca.events.block_found"),
      cell_op(block->dloc()),
      m_block(block) {}

/*******************************************************************************
 * Depth0 Foraging
 ******************************************************************************/
void block_found::visit(ds::dpo_store& store) {
  /*
   * If the cell in the arena that we thought contained a cache now contains a
   * block, remove the out-of-date cache.
   */
  for (auto& c : store.caches().values_range()) {
    if (m_block->dloc() == c.ent()->dloc()) {
      store.cache_remove(c.ent_obj());

      /*
       * We need to start a new decay count because the type of object in a
       * given cell has changed. This is regardless of the status repeat
       * deposits, which only affect repeated sightings of KNOWN objects.
       */
      c.density().reset();
    }
  } /* for(&&c..) */

  rswarm::pheromone_density density(store.pheromone_rho());
  auto known = store.find(m_block);
  if (nullptr != known) {
    /*
     * If the block we just "found" is already known and has a different
     * location than we think it should, then our tracked version is out of date
     * and needs to be removed.
     */
    if (!known->ent()->dloccmp(*m_block)) {
      ER_INFO("Removing block%d@%s: Moved to %s",
              known->ent()->id(),
              known->ent()->dloc().to_str().c_str(),
              m_block->dloc().to_str().c_str());
      store.block_remove(known->ent_obj());
      density.pheromone_set(ds::dpo_store::kNRD_MAX_PHEROMONE);
    } else { /* block has not moved */
      density = known->density();

      /*
       * If repeat pheromon deposits are enabled, make a deposit. Otherwise,
       * just reset the pheromone density to make because we have seen the block
       * again.
       */
      if (store.repeat_deposit()) {
        density.pheromone_add(rswarm::pheromone_density::kUNIT_QUANTITY);
      } else {
        density.pheromone_set(ds::dpo_store::kNRD_MAX_PHEROMONE);
      }
    }
  } else {
    density.pheromone_set(ds::dpo_store::kNRD_MAX_PHEROMONE);
  }

  store.block_update(ds::dp_block_map::value_type(m_block, density));
} /* visit() */

/*******************************************************************************
 * MDPO Foraging
 ******************************************************************************/
void block_found::visit(ds::cell2D& cell) {
  cell.entity(m_block);
  this->visit(cell.fsm());
} /* visit() */

void block_found::visit(fsm::cell2D_fsm& fsm) {
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

void block_found::visit(ds::dpo_semantic_map& map) {
  ds::cell2D& cell = map.access<occupancy_grid::kCell>(x(), y());
  rswarm::pheromone_density& density =
      map.access<occupancy_grid::kPheromone>(x(), y());

  if (!cell.state_is_known()) {
    map.known_cells_inc();
    ER_ASSERT(map.known_cell_count() <= map.xdsize() * map.ydsize(),
              "Known cell count (%u) >= arena dimensions (%zux%zu)",
              map.known_cell_count(),
              map.xdsize(),
              map.ydsize());
  }
  /*
   * If the cell is currently in a HAS_CACHE state, then that means that this
   * cell is coming back into our LOS with a block, when it contained a cache
   * the last time it was seen. Remove the cache/synchronize with reality.
   *
   * The density needs to be reset as well, as we are now tracking a different
   * kind of cell entity.
   */
  if (cell.state_has_cache()) {
    map.cache_remove(cell.cache());
  }

  /*
   * If the ID of the block we currently think resides in the cell and the ID of
   * the one we just found that actually resides there are not the same, we need
   * to reset the density for the cell, and start a new decay count.
   */
  if (cell.state_has_block() && cell.block()->id() != m_block->id()) {
    density.reset();
  }

  pheromone_update(map);

  ER_ASSERT(cell.state_has_block(),
            "Cell@%s not in HAS_BLOCK",
            cell.loc().to_str().c_str());
  ER_ASSERT(cell.block()->id() == m_block->id(),
            "Block for cell@%s ID mismatch: %d/%d",
            cell.loc().to_str().c_str(),
            m_block->id(),
            cell.block()->id());
} /* visit() */

void block_found::pheromone_update(ds::dpo_semantic_map& map) {
  rswarm::pheromone_density& density =
      map.access<occupancy_grid::kPheromone>(x(), y());
  ds::cell2D& cell = map.access<occupancy_grid::kCell>(x(), y());
  if (map.pheromone_repeat_deposit()) {
    density.pheromone_add(rswarm::pheromone_density::kUNIT_QUANTITY);
  } else {
    /*
     * Seeing a new block on empty square or one that used to contain a cache.
     */
    if (!cell.state_has_block()) {
      density.reset();
      density.pheromone_add(rswarm::pheromone_density::kUNIT_QUANTITY);
    } else { /* Seeing a known block again--set its relevance to the max */
      density.pheromone_set(ds::dpo_store::kNRD_MAX_PHEROMONE);
    }
  }
  /*
   * ONLY if the underlying DPO store actually changed do we update what block
   * the cell points to. If we do it unconditionally, we are left
   * with dangling references as a result of mixing unique_ptr and raw ptr. See
   * #229.
   */
  auto res = map.block_update(ds::dp_block_map::value_type(m_block, density));
  if (res.status) {
    if (ds::dpo_store::update_status::kBLOCK_MOVED == res.reason) {
      ER_DEBUG("Updating cell@%s: Block%d moved %s -> %s",
               res.old_loc.to_str().c_str(),
               m_block->id(),
               res.old_loc.to_str().c_str(),
               m_block->dloc().to_str().c_str());
      events::cell_empty_visitor op(res.old_loc);
      op.visit(map.access<occupancy_grid::kCell>(res.old_loc));
    } else {
      ER_ASSERT(ds::dpo_store::update_status::kNEW_BLOCK_ADDED == res.reason,
                "Bad reason for DPO store update: %d",
                res.reason);
    }

    /*
     * At this point we know that if the block was previously tracked, its old
     * host cell has been updated and the block updated in the store, so we are
     * good to update the NEW host cell to point to the block.
     */
    visit(cell);
  }
} /* pheromone_update() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void block_found::visit(controller::depth2::grp_mdpo_controller& c) {
  c.ndc_push();

  visit(*c.mdpo_perception()->map());

  c.ndc_pop();
} /* visit() */

void block_found::visit(controller::depth2::grp_dpo_controller& c) {
  c.ndc_push();

  visit(*c.dpo_perception()->dpo_store());

  c.ndc_pop();
} /* visit() */

void block_found::visit(controller::depth2::grp_omdpo_controller& c) {
  c.ndc_push();

  visit(*c.mdpo_perception()->map());

  c.ndc_pop();
} /* visit() */

void block_found::visit(controller::depth2::grp_odpo_controller& c) {
  c.ndc_push();

  visit(*c.dpo_perception()->dpo_store());

  c.ndc_pop();
} /* visit() */

NS_END(detail, events, fordyca);
