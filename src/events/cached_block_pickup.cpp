/**
 * @file cached_block_pickup.cpp
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
#include "fordyca/events/cached_block_pickup.hpp"

#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/block_to_goal_fsm.hpp"
#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/cache_collector.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);
using ds::arena_grid;
using ds::occupancy_grid;
using representation::base_cache;
namespace rfsm = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(
    const std::shared_ptr<representation::arena_cache>& cache,
    uint robot_index,
    uint timestep)
    : cell_op(cache->discrete_loc()),
      ER_CLIENT_INIT("fordyca.events.cached_block_pickup"),
      m_robot_index(robot_index),
      m_timestep(timestep),
      m_real_cache(cache) {
  ER_ASSERT(m_real_cache->n_blocks() >= base_cache::kMinBlocks,
            "< %zu blocks in cache",
            base_cache::kMinBlocks);
  m_pickup_block = m_real_cache->oldest_block();
  ER_ASSERT(m_pickup_block, "No block in non-empty cache");
}

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cached_block_pickup::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void cached_block_pickup::visit(ds::cell2D& cell) {
  ER_ASSERT(0 != cell.loc().x() && 0 != cell.loc().y(),
            "Cell does not have coordinates");
  ER_ASSERT(cell.state_has_cache(), "Cell does not have cache");
  if (nullptr != m_orphan_block) {
    cell.entity(m_orphan_block);
    ER_DEBUG("Cell (%u, %u) gets orphan block%d",
             cell_op::x(),
             cell_op::y(),
             m_orphan_block->id());
  }
  cell.fsm().accept(*this);
} /* visit() */

void cached_block_pickup::visit(representation::arena_cache& cache) {
  cache.block_remove(m_pickup_block);
  cache.has_block_pickup();
} /* visit() */

void cached_block_pickup::visit(ds::arena_map& map) {
  ER_ASSERT(m_real_cache->n_blocks() >= base_cache::kMinBlocks,
            "< %zu blocks in cache",
            base_cache::kMinBlocks);
  int cache_id = m_real_cache->id();
  ER_ASSERT(-1 != cache_id, "Cache ID undefined on block pickup");

  rmath::vector2u cache_coord = m_real_cache->discrete_loc();
  ER_ASSERT(cache_coord == cell_op::coord(),
            "Coordinates for cache%d%s/cell@%s do not agree",
            cache_id,
            cache_coord.to_str().c_str(),
            cell_op::coord().to_str().c_str());

  ds::cell2D& cell = map.access<arena_grid::kCell>(cell_op::x(), cell_op::y());
  ER_ASSERT(m_real_cache->n_blocks() == cell.block_count(),
            "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
            m_real_cache->n_blocks(),
            cell.block_count());

  /*
   * If there are more than kMinBlocks blocks in cache, just remove one, and
   * update the underlying cell. If there are only kMinBlocks left, do the same
   * thing but also remove the cache, as a cache with less than that many blocks
   * is not a cache.
   */
  if (m_real_cache->n_blocks() > base_cache::kMinBlocks) {
    m_real_cache->accept(*this);
    cell.accept(*this);
    ER_ASSERT(cell.state_has_cache(),
              "Cell@(%u, %u) with >= %zu blocks does not have cache",
              cell_op::x(),
              cell_op::y(),
              base_cache::kMinBlocks);

    ER_INFO(
        "arena_map: fb%u: block%d from cache%d@(%u, %u),remaining=[%s] (%zu)",
        m_robot_index,
        m_pickup_block->id(),
        cache_id,
        cell_op::x(),
        cell_op::y(),
        rcppsw::to_string(m_real_cache->blocks()).c_str(),
        m_real_cache->n_blocks());

  } else {
    m_real_cache->accept(*this);
    m_orphan_block = m_real_cache->oldest_block();
    cell.accept(*this);

    ER_ASSERT(cell.state_has_block(),
              "cell@(%u, %u) with 1 block has cache",
              cell_op::x(),
              cell_op::y());

    map.cache_extent_clear(m_real_cache);
    map.cache_remove(m_real_cache);
    map.caches_removed(1);
    ER_INFO("arena_map: fb%u: block%d from cache%d@(%u, %u) [depleted]",
            m_robot_index,
            m_pickup_block->id(),
            cache_id,
            cell_op::x(),
            cell_op::y());
  }
  m_pickup_block->accept(*this);
} /* visit() */

void cached_block_pickup::visit(ds::dpo_store& store) {
  ER_ASSERT(store.contains(m_real_cache),
            "Cache%d@%s not in DPO store",
            m_real_cache->id(),
            m_real_cache->discrete_loc().to_str().c_str());

  auto pcache = store.find(m_real_cache);
  ER_ASSERT(pcache->ent()->contains_block(m_pickup_block),
            "DPO cache%d@%s/%s does not contain pickup block%d",
            pcache->ent()->id(),
            pcache->ent()->real_loc().to_str().c_str(),
            pcache->ent()->discrete_loc().to_str().c_str(),
            m_pickup_block->id());

  if (pcache->ent()->n_blocks() > base_cache::kMinBlocks) {
    pcache->ent_obj()->block_remove(m_pickup_block);
    ER_INFO("DPO Store: fb%u: block%d from cache%d@%s,remaining=[%s] (%zu)",
            m_robot_index,
            m_pickup_block->id(),
            pcache->ent()->id(),
            cell_op::coord().to_str().c_str(),
            rcppsw::to_string(pcache->ent()->blocks()).c_str(),
            pcache->ent()->n_blocks());

  } else {
    __rcsw_unused int id = pcache->ent()->id();
    pcache->ent_obj()->block_remove(m_pickup_block);
    store.cache_remove(pcache->ent_obj());
    ER_INFO("DPO Store: fb%u: block%d from cache%d@%s [depleted]",
            m_robot_index,
            m_pickup_block->id(),
            id,
            cell_op::coord().to_str().c_str());
  }
} /* visit() */

void cached_block_pickup::visit(ds::dpo_semantic_map& map) {
  ds::cell2D& cell = map.access<occupancy_grid::kCell>(cell_op::coord());
  ER_ASSERT(cell.state_has_cache(), "Cell does not contain cache");
  ER_ASSERT(cell.cache()->n_blocks() == cell.block_count(),
            "Perceived cache/cell disagree on # of blocks: "
            "cache=%zu/cell=%zu",
            cell.cache()->n_blocks(),
            cell.block_count());

  ER_ASSERT(cell.cache()->contains_block(m_pickup_block),
            "Perceived cache%d@%s/%s does not contain pickup block%d",
            cell.cache()->id(),
            cell.cache()->real_loc().to_str().c_str(),
            cell.cache()->discrete_loc().to_str().c_str(),
            m_pickup_block->id());

  if (cell.cache()->n_blocks() > base_cache::kMinBlocks) {
    cell.cache()->block_remove(m_pickup_block);
    cell.accept(*this);
    ER_ASSERT(cell.state_has_cache(),
              "cell@%s with >= 2 blocks does not have cache",
              cell_op::coord().to_str().c_str());

    ER_INFO("DPO Map: fb%u: block%d from cache%d@%s,remaining=[%s] (%zu)",
            m_robot_index,
            m_pickup_block->id(),
            cell.cache()->id(),
            cell_op::coord().to_str().c_str(),
            rcppsw::to_string(cell.cache()->blocks()).c_str(),
            cell.cache()->n_blocks());

  } else {
    __rcsw_unused int id = cell.cache()->id();
    cell.cache()->block_remove(m_pickup_block);

    map.cache_remove(cell.cache());
    ER_INFO("DPO Map: fb%u: block%d from cache%d@%s [depleted]",
            m_robot_index,
            m_pickup_block->id(),
            id,
            cell_op::coord().to_str().c_str());
  }
} /* visit() */

void cached_block_pickup::visit(representation::base_block& block) {
  ER_ASSERT(-1 != block.id(), "Unamed block");
  block.add_transporter(m_robot_index);
  block.first_pickup_time(m_timestep);

  block.move_out_of_sight();
  ER_INFO("Block%d is now carried by fb%u", block.id(), m_robot_index);
} /* visit() */

void cached_block_pickup::visit(
    controller::depth1::gp_dpo_controller& controller) {
  controller.ndc_push();

  static_cast<controller::dpo_perception_subsystem*>(controller.perception())
      ->store()
      ->accept(*this);
  controller.block(m_pickup_block);

  auto* task = dynamic_cast<events::existing_cache_interactor*>(
      controller.current_task());
  std::string task_name = dynamic_cast<ta::logical_task*>(task)->name();

  ER_ASSERT(
      nullptr != task,
      "Non existing cache interactor task '%s' causing cached block pickup",
      task_name.c_str());
  task->accept(*this);

  ER_INFO("Picked up block%d from cache%d,task='%s'",
          m_pickup_block->id(),
          m_real_cache->id(),
          task_name.c_str());

  controller.ndc_pop();
} /* visit() */

void cached_block_pickup::visit(
    controller::depth1::gp_mdpo_controller& controller) {
  controller.ndc_push();

  controller.perception()->map()->accept(*this);
  controller.block(m_pickup_block);

  auto* task = dynamic_cast<events::existing_cache_interactor*>(
      controller.current_task());
  std::string task_name = dynamic_cast<ta::logical_task*>(task)->name();

  ER_ASSERT(
      nullptr != task,
      "Non existing cache interactor task '%s' causing cached block pickup",
      task_name.c_str());
  task->accept(*this);

  ER_INFO("Picked up block%d from cache%d,task='%s'",
          m_pickup_block->id(),
          m_real_cache->id(),
          task_name.c_str());

  controller.ndc_pop();
} /* visit() */

void cached_block_pickup::visit(tasks::depth1::collector& task) {
  static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism())
      ->accept(*this);
} /* visit() */

void cached_block_pickup::visit(fsm::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   rfsm::event_type::NORMAL);
} /* visit() */

void cached_block_pickup::visit(fsm::depth1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   rfsm::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cached_block_pickup::visit(
    controller::depth2::grp_mdpo_controller& controller) {
  controller.ndc_push();

  static_cast<controller::mdpo_perception_subsystem*>(controller.perception())
      ->map()
      ->accept(*this);
  controller.block(m_pickup_block);

  auto* polled = dynamic_cast<ta::polled_task*>(controller.current_task());
  auto* interactor = dynamic_cast<events::existing_cache_interactor*>(
      controller.current_task());

  ER_ASSERT(nullptr != interactor,
            "Non existing cache interactor task %s causing cached block pickup",
            polled->name().c_str());

  if (tasks::depth2::foraging_task::kCacheTransfererName == polled->name()) {
    ER_INFO("Added cache%d@%s to drop exception list,task='%s'",
            m_real_cache->id(),
            m_real_cache->real_loc().to_str().c_str(),
            polled->name().c_str());
    controller.cache_sel_matrix()->sel_exception_add(
        {m_real_cache->id(), controller::cache_sel_exception::kDrop});
    controller.csel_exception_added(true);
  }
  interactor->accept(*this);
  ER_INFO("Picked up block%d from cache%d,task='%s'",
          m_pickup_block->id(),
          m_real_cache->id(),
          polled->name().c_str());

  controller.ndc_pop();
} /* visit() */

void cached_block_pickup::visit(tasks::depth2::cache_transferer& task) {
  static_cast<fsm::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */
void cached_block_pickup::visit(tasks::depth2::cache_collector& task) {
  static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism())
      ->accept(*this);
} /* visit() */

NS_END(events, fordyca);
