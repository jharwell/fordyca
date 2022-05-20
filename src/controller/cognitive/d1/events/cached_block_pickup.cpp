/**
 * \file cached_block_pickup.cpp
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
#include "fordyca/controller/cognitive/d1/events/cached_block_pickup.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_semantic_map.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/tasks/d1/collector.hpp"
#include "fordyca/tasks/d1/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

using carepr::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(const carepr::arena_cache* cache,
                                         crepr::base_block3D* block,
                                         const rtypes::type_uuid& id,
                                         const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.controller.cognitive.d1.events.cached_block_"
                     "pickup"),
      base_block_pickup(block, id, t),
      mc_timestep(t),
      mc_cache(cache) {}

cached_block_pickup::~cached_block_pickup(void) = default;

/*******************************************************************************
 * Controllers
 ******************************************************************************/
void cached_block_pickup::visit(fccd1::bitd_dpo_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.perception()->model<fspds::dpo_store>());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cached_block_pickup::visit(fccd1::bitd_mdpo_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.perception()->model<fspds::dpo_store>());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cached_block_pickup::visit(fccd1::bitd_odpo_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.perception()->model<fspds::dpo_store>());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

void cached_block_pickup::visit(fccd1::bitd_omdpo_controller& controller) {
  controller.ndc_uuid_push();

  visit(*controller.perception()->model<fspds::dpo_store>());
  visit(static_cast<ccontroller::block_carrying_controller&>(controller));
  dispatch_cache_interactor(controller.current_task());

  controller.ndc_uuid_pop();
} /* visit() */

/*******************************************************************************
 * FSMs
 ******************************************************************************/
void cached_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void cached_block_pickup::visit(fsm::d1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void cached_block_pickup::visit(cfsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

/*******************************************************************************
 * Data Structures
 ******************************************************************************/
void cached_block_pickup::visit(cds::cell2D& cell) {
  visit(cell.fsm());
} /* visit() */

void cached_block_pickup::visit(fspds::dpo_store& store) {
  ER_ASSERT(store.contains(mc_cache),
            "Cache%d@%s/%s not in DPO store",
            mc_cache->id().v(),
            rcppsw::to_string(mc_cache->rcenter2D()).c_str(),
            rcppsw::to_string(mc_cache->dcenter2D()).c_str());

  auto* pcache = store.find(mc_cache);

  /*
   * In general this should be true, but it cannot be an assert, because for
   * oracular controllers LOS processing happens BEFORE arena interactions, and
   * they may have their internal object store updated to reflect the depleted
   * cache state.
   *
   * This is not an issue for blocks, because a block that has moved/disappeared
   * via oracular information injection vanishes from the robot's perception.
   */
  if (!pcache->ent()->contains_block(block())) {
    ER_INFO("DPO cache%d@%s/%s does not contain pickup block%d",
            pcache->ent()->id().v(),
            rcppsw::to_string(pcache->ent()->rcenter2D()).c_str(),
            rcppsw::to_string(pcache->ent()->dcenter2D()).c_str(),
            block()->id().v());
    return;
  }

  if (pcache->ent()->n_blocks() > base_cache::kMinBlocks) {
    pcache->ent()->block_remove(block());
    ER_INFO("DPO Store: block%d from cache%d@%s,remaining=[%s] (%zu)",
            block()->id().v(),
            pcache->ent()->id().v(),
            rcppsw::to_string(block()->danchor2D()).c_str(),
            rcppsw::to_string(pcache->ent()->blocks()).c_str(),
            pcache->ent()->n_blocks());

  } else {
    RCPPSW_UNUSED rtypes::type_uuid id = pcache->ent()->id();
    pcache->ent()->block_remove(block());
    store.cache_remove(pcache->ent());
    ER_INFO("DPO Store: block%d from cache%d@%s [depleted]",
            block()->id().v(),
            id.v(),
            rcppsw::to_string(block()->danchor2D()).c_str());
  }
} /* visit() */

void cached_block_pickup::visit(fspds::dpo_semantic_map& map) {
  auto& cell = map.access<fspds::occupancy_grid::kCell>(block()->danchor2D());
  ER_ASSERT(cell.state_has_cache(), "Cell does not contain cache");
  ER_ASSERT(cell.cache()->n_blocks() == cell.block_count(),
            "Perceived cache/cell disagree on # of blocks: "
            "cache=%zu/cell=%zu",
            cell.cache()->n_blocks(),
            cell.block_count());

  ER_ASSERT(cell.cache()->contains_block(block()),
            "Perceived cache%d@%s/%s does not contain pickup block%d",
            cell.cache()->id().v(),
            rcppsw::to_string(cell.cache()->rcenter2D()).c_str(),
            rcppsw::to_string(cell.cache()->dcenter2D()).c_str(),
            block()->id().v());

  if (cell.cache()->n_blocks() > base_cache::kMinBlocks) {
    cell.cache()->block_remove(block());
    visit(cell);
    ER_ASSERT(cell.state_has_cache(),
              "cell@%s with >= 2 blocks does not have cache",
              rcppsw::to_string(block()->danchor2D()).c_str());

    ER_INFO("DPO Map: block%d from cache%d@%s,remaining=[%s] (%zu)",
            block()->id().v(),
            cell.cache()->id().v(),
            rcppsw::to_string(block()->danchor2D()).c_str(),
            rcppsw::to_string(cell.cache()->blocks()).c_str(),
            cell.cache()->n_blocks());

  } else {
    RCPPSW_UNUSED rtypes::type_uuid id = cell.cache()->id();
    cell.cache()->block_remove(block());

    map.cache_remove(cell.cache());
    ER_INFO("DPO Map: block%d from cache%d@%s [depleted]",
            block()->id().v(),
            id.v(),
            rcppsw::to_string(block()->danchor2D()).c_str());
  }
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cached_block_pickup::dispatch_cache_interactor(
    tasks::base_foraging_task* const task) {
  auto* interactor = dynamic_cast<fevents::existing_cache_interactor*>(task);
  std::string task_name = dynamic_cast<cta::logical_task*>(task)->name();
  ER_ASSERT(nullptr != task,
            "Non existing cache interactor task '%s' causing cached block pickup",
            task_name.c_str());
  interactor->accept(*this);
  ER_INFO("Picked up block%d from cache%d,task='%s'",
          block()->id().v(),
          mc_cache->id().v(),
          task_name.c_str());
} /* dispatch_cache_interactor() */

crepr::sim_block3D* cached_block_pickup::block(void) {
  return static_cast<crepr::sim_block3D*>(ccops::base_block_pickup::block());
}

NS_END(events, d1, cognitive, controller, fordyca);
