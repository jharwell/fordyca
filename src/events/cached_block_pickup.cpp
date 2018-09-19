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

#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_goal_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/foraging_task.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);
using ds::arena_grid;
using ds::occupancy_grid;
using representation::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(
    const std::shared_ptr<representation::arena_cache>& cache,
    uint robot_index,
    uint timestep)
    : cell_op(cache->discrete_loc().first, cache->discrete_loc().second),
      ER_CLIENT_INIT("fordyca.events.cached_block_pickup"),
      m_robot_index(robot_index),
      m_timestep(timestep),
      m_real_cache(cache) {
  ER_ASSERT(m_real_cache->n_blocks() >= base_cache::kMinBlocks,
            "< %zu blocks in cache",
            base_cache::kMinBlocks);
  m_pickup_block = m_real_cache->block_get();
  ER_ASSERT(m_pickup_block, "No block in non-empty cache");
}

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cached_block_pickup::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void cached_block_pickup::visit(ds::cell2D& cell) {
  ER_ASSERT(0 != cell.loc().first && 0 != cell.loc().second,
            "Cell does not have coordinates");
  ER_ASSERT(cell.state_has_cache(), "cell does not have cache");
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

  rcppsw::math::dcoord2 coord = m_real_cache->discrete_loc();
  ER_ASSERT(coord == rcppsw::math::dcoord2(cell_op::x(), cell_op::y()),
            "Coordinates for cache%d (%u, %u)/cell(%u, %u) do not "
            "agree",
            cache_id,
            coord.first,
            coord.second,
            cell_op::y(),
            cell_op::y());

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
              "cell@(%u, %u) with >= %zu blocks does not have cache",
              cell_op::x(),
              cell_op::y(),
              base_cache::kMinBlocks);
    ER_INFO(
        "arena_map: fb%u: block%d from cache%d@(%u, %u) [%zu blocks remain]",
        m_robot_index,
        m_pickup_block->id(),
        cache_id,
        cell_op::x(),
        cell_op::y(),
        m_real_cache->n_blocks());

  } else {
    m_real_cache->accept(*this);
    m_orphan_block = m_real_cache->block_get();
    cell.accept(*this);

    ER_ASSERT(cell.state_has_block(),
              "cell@(%u, %u) with 1 block has cache",
              cell_op::x(),
              cell_op::y());

    map.cache_extent_clear(m_real_cache);
    map.cache_remove(m_real_cache);
    map.caches_removed(1);
    ER_INFO("arena_map: fb%u: block%d from cache%d@(%u, %u) [cache depleted]",
            m_robot_index,
            m_pickup_block->id(),
            cache_id,
            cell_op::x(),
            cell_op::y());
  }
  m_pickup_block->accept(*this);
} /* visit() */

void cached_block_pickup::visit(ds::perceived_arena_map& map) {
  ds::cell2D& cell =
      map.access<occupancy_grid::kCell>(cell_op::x(), cell_op::y());
  ER_ASSERT(cell.state_has_cache(), "Cell does not contain cache");
  ER_ASSERT(cell.cache()->n_blocks() == cell.block_count(),
            "Perceived cache/cell disagree on # of blocks: "
            "cache=%zu/cell=%zu",
            cell.cache()->n_blocks(),
            cell.block_count());

  ER_ASSERT(cell.cache()->contains_block(m_pickup_block),
            "Perceived cache does not contain ref to block to be picked up");

  if (cell.cache()->n_blocks() > base_cache::kMinBlocks) {
    cell.cache()->block_remove(m_pickup_block);
    cell.accept(*this);
    ER_ASSERT(cell.state_has_cache(),
              "cell@(%u, %u) with >= 2 blocks does not have cache",
              cell_op::x(),
              cell_op::y());
    ER_INFO(
        "perceived_arena_map: fb%u: block%d from cache%d@(%u, %u) [%zu "
        "blocks remain]",
        m_robot_index,
        m_pickup_block->id(),
        cell.cache()->id(),
        cell_op::x(),
        cell_op::y(),
        cell.cache()->n_blocks());

  } else {
    __rcsw_unused int id = cell.cache()->id();
    cell.cache()->block_remove(m_pickup_block);

    map.cache_remove(cell.cache());
    ER_INFO("PAM: fb%u: block%d from cache%d@(%u, %u) [cache depleted]",
            m_robot_index,
            m_pickup_block->id(),
            id,
            cell_op::x(),
            cell_op::y());
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
    controller::depth1::foraging_controller& controller) {
  controller.ndc_push();
  controller.perception()->map()->accept(*this);
  controller.block(m_pickup_block);
  dynamic_cast<tasks::depth1::existing_cache_interactor*>(
      controller.current_task())
      ->accept(*this);

  ER_INFO("Picked up block%d", m_pickup_block->id());
  controller.ndc_pop();
} /* visit() */

void cached_block_pickup::visit(tasks::depth1::collector& task) {
  static_cast<fsm::depth1::cached_block_to_nest_fsm*>(task.mechanism())
      ->accept(*this);
} /* visit() */

void cached_block_pickup::visit(fsm::depth1::block_to_goal_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   state_machine::event_type::NORMAL);
} /* visit() */

void cached_block_pickup::visit(fsm::depth1::cached_block_to_nest_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   state_machine::event_type::NORMAL);
} /* visit() */

/*******************************************************************************
 * Depth2 Foraging
 ******************************************************************************/
void cached_block_pickup::visit(
    controller::depth2::foraging_controller& controller) {
  ER_ASSERT(false, "Not implemented");
} /* visit() */

void cached_block_pickup::visit(tasks::depth2::cache_transferer& task) {
  static_cast<fsm::depth1::block_to_goal_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

NS_END(events, fordyca);
