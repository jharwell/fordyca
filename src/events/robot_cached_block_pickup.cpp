/**
 * \file robot_cached_block_pickup.cpp
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
#include "fordyca/events/robot_cached_block_pickup.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/base_block3D.hpp"

#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/foraging_signal.hpp"
#include "fordyca/support/base_cache_manager.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/cache_collector.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);
using carepr::base_cache;
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
robot_cached_block_pickup::robot_cached_block_pickup(
    const carepr::arena_cache* cache,
    const crepr::base_block3D* block,
    const rtypes::type_uuid& robot_id,
    const rtypes::timestep& t)
    : ER_CLIENT_INIT("fordyca.events.robot_cached_block_pickup"),
      cell2D_op(cache->dpos2D()),
      mc_robot_id(robot_id),
      mc_timestep(t),
      mc_cache(cache),
      mc_block(block),
      m_robot_block(nullptr) {}

robot_cached_block_pickup::~robot_cached_block_pickup(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void robot_cached_block_pickup::dispatch_d1_cache_interactor(
    tasks::base_foraging_task* const task) {
  auto* interactor = dynamic_cast<events::existing_cache_interactor*>(task);
  std::string task_name = dynamic_cast<cta::logical_task*>(task)->name();
  ER_ASSERT(
      nullptr != task,
      "Non existing cache interactor task '%s' causing cached block pickup",
      task_name.c_str());
  interactor->accept(*this);
  ER_INFO("Picked up block%d from cache%d,task='%s'",
          mc_block->id().v(),
          mc_cache->id().v(),
          task_name.c_str());
} /* dispatch_d1_cache_interactor() */

bool robot_cached_block_pickup::dispatch_d2_cache_interactor(
    tasks::base_foraging_task* task,
    controller::cache_sel_matrix* csel_matrix) {
  auto* polled = dynamic_cast<cta::polled_task*>(task);
  auto* interactor = dynamic_cast<events::existing_cache_interactor*>(task);
  bool ret = false;

  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s causing cached block pickup",
            polled->name().c_str());

  if (tasks::depth2::foraging_task::kCacheTransfererName == polled->name()) {
    ER_INFO("Added cache%d@%s to drop exception list,task='%s'",
            mc_cache->id().v(),
            mc_cache->rpos2D().to_str().c_str(),
            polled->name().c_str());
    csel_matrix->sel_exception_add(
        {mc_cache->id(), controller::cache_sel_exception::ekDROP});
    ret = true;
  }
  interactor->accept(*this);
  ER_INFO("Picked up block%d from cache%d,task='%s'",
          mc_block->id().v(),
          mc_cache->id().v(),
          polled->name().c_str());
  return ret;
} /* dispatch_d2_cache_interactor() */

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void robot_cached_block_pickup::visit(cfsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void robot_cached_block_pickup::visit(cds::cell2D& cell) {
  visit(cell.fsm());
} /* visit() */

void robot_cached_block_pickup::visit(ds::dpo_store& store) {
  ER_ASSERT(store.contains(mc_cache),
            "Cache%d@%s not in DPO store",
            mc_cache->id().v(),
            mc_cache->dpos2D().to_str().c_str());

  auto pcache = store.find(mc_cache);

  /*
   * In general this should be true, but it cannot be an assert, because for
   * oracular controllers LOS processing happens BEFORE arena interactions, and
   * they may have their internal object store updated to reflect the depleted
   * cache state.
   *
   * This is not an issue for blocks, because a block that has moved/disappeared
   * via oracular information injection vanishes from the robot's perception.
   */
  if (!pcache->ent()->contains_block(mc_block)) {
    ER_INFO("DPO cache%d@%s/%s does not contain pickup block%d",
            pcache->ent()->id().v(),
            pcache->ent()->rpos2D().to_str().c_str(),
            pcache->ent()->dpos2D().to_str().c_str(),
            mc_block->id().v());
    return;
  }

  if (pcache->ent()->n_blocks() > base_cache::kMinBlocks) {
    pcache->ent()->block_remove(m_robot_block.get());
    ER_INFO("DPO Store: fb%u: block%d from cache%d@%s,remaining=[%s] (%zu)",
            mc_robot_id.v(),
            mc_block->id().v(),
            pcache->ent()->id().v(),
            cell2D_op::coord().to_str().c_str(),
            rcppsw::to_string(pcache->ent()->blocks()).c_str(),
            pcache->ent()->n_blocks());

  } else {
    RCSW_UNUSED rtypes::type_uuid id = pcache->ent()->id();
    pcache->ent()->block_remove(m_robot_block.get());
    store.cache_remove(pcache->ent());
    ER_INFO("DPO Store: fb%u: block%d from cache%d@%s [depleted]",
            mc_robot_id.v(),
            mc_block->id().v(),
            id.v(),
            cell2D_op::coord().to_str().c_str());
  }
} /* visit() */

void robot_cached_block_pickup::visit(ds::dpo_semantic_map& map) {
  cds::cell2D& cell = map.access<occupancy_grid::kCell>(cell2D_op::coord());
  ER_ASSERT(cell.state_has_cache(), "Cell does not contain cache");
  ER_ASSERT(cell.cache()->n_blocks() == cell.block_count(),
            "Perceived cache/cell disagree on # of blocks: "
            "cache=%zu/cell=%zu",
            cell.cache()->n_blocks(),
            cell.block_count());

  ER_ASSERT(cell.cache()->contains_block(mc_block),
            "Perceived cache%d@%s/%s does not contain pickup block%d",
            cell.cache()->id().v(),
            cell.cache()->rpos2D().to_str().c_str(),
            cell.cache()->dpos2D().to_str().c_str(),
            mc_block->id().v());

  if (cell.cache()->n_blocks() > base_cache::kMinBlocks) {
    cell.cache()->block_remove(m_robot_block.get());
    visit(cell);
    ER_ASSERT(cell.state_has_cache(),
              "cell@%s with >= 2 blocks does not have cache",
              cell2D_op::coord().to_str().c_str());

    ER_INFO("DPO Map: fb%u: block%d from cache%d@%s,remaining=[%s] (%zu)",
            mc_robot_id.v(),
            mc_block->id().v(),
            cell.cache()->id().v(),
            cell2D_op::coord().to_str().c_str(),
            rcppsw::to_string(cell.cache()->blocks()).c_str(),
            cell.cache()->n_blocks());

  } else {
    RCSW_UNUSED rtypes::type_uuid id = cell.cache()->id();
    cell.cache()->block_remove(m_robot_block.get());

    map.cache_remove(cell.cache());
    ER_INFO("DPO Map: fb%u: block%d from cache%d@%s [depleted]",
            mc_robot_id.v(),
            mc_block->id().v(),
            id.v(),
            cell2D_op::coord().to_str().c_str());
  }
} /* visit() */

void robot_cached_block_pickup::visit(support::base_cache_manager& manager) {
  if (mc_cache->n_blocks() == base_cache::kMinBlocks) {
    std::scoped_lock lock(manager.mtx());
    manager.cache_depleted(mc_timestep - mc_cache->creation_ts());
  }
} /* visit() */

void robot_cached_block_pickup::visit(
    controller::depth1::bitd_dpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the store so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.dpo_perception()->dpo_store());
  controller.block(std::move(m_robot_block));

  dispatch_d1_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(
    controller::depth1::bitd_mdpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the map so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.mdpo_perception()->map());
  controller.block(std::move(m_robot_block));

  dispatch_d1_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(
    controller::depth1::bitd_odpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the store so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.dpo_perception()->dpo_store());
  controller.block(std::move(m_robot_block));

  dispatch_d1_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(
    controller::depth1::bitd_omdpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the map so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.mdpo_perception()->map());
  controller.block(std::move(m_robot_block));

  dispatch_d1_cache_interactor(controller.current_task());

  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(tasks::depth1::collector& task) {
  visit(*static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

void robot_cached_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

void robot_cached_block_pickup::visit(fsm::depth1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(fsm::foraging_signal::ekBLOCK_PICKUP,
                   rpfsm::event_type::ekNORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void robot_cached_block_pickup::visit(
    controller::depth2::birtd_dpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the store so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.dpo_perception()->dpo_store());
  controller.block(std::move(m_robot_block));

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }

  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(
    controller::depth2::birtd_mdpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the map so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.mdpo_perception()->map());
  controller.block(std::move(m_robot_block));

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(
    controller::depth2::birtd_odpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the store so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.dpo_perception()->dpo_store());
  controller.block(std::move(m_robot_block));

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }

  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(
    controller::depth2::birtd_omdpo_controller& controller) {
  controller.ndc_pusht();

  /*
   * Cloning must be before visiting the map so the C++ semantics of
   * block/cache removal work out.
   */
  m_robot_block = mc_block->clone();
  m_robot_block->md()->robot_id(mc_robot_id);

  visit(*controller.mdpo_perception()->map());
  controller.block(std::move(m_robot_block));

  if (dispatch_d2_cache_interactor(controller.current_task(),
                                   controller.cache_sel_matrix())) {
    controller.csel_exception_added(true);
  }
  controller.ndc_pop();
} /* visit() */

void robot_cached_block_pickup::visit(tasks::depth2::cache_transferer& task) {
  visit(*static_cast<fsm::block_to_goal_fsm*>(task.mechanism()));
} /* visit() */
void robot_cached_block_pickup::visit(tasks::depth2::cache_collector& task) {
  visit(*static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism()));
} /* visit() */

NS_END(detail, events, fordyca);
