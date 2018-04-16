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
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/events/cache_found.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/fsm/cell2D_fsm.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_cache_fsm.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"

#include "fordyca/tasks/collector.hpp"
#include "fordyca/tasks/foraging_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);
using representation::base_cache;
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(
    const std::shared_ptr<rcppsw::er::server>& server,
    const std::shared_ptr<representation::arena_cache>& cache,
    size_t robot_index)
    : cell_op(cache->discrete_loc().first, cache->discrete_loc().second),
      client(server),
      m_robot_index(robot_index),
      m_real_cache(cache),
      m_server(server) {
  client::insmod("cached_block_pickup",
                 rcppsw::er::er_lvl::DIAG,
                 rcppsw::er::er_lvl::NOM);
  ER_ASSERT(m_real_cache->n_blocks() >= 2, "FATAL: < 2 blocks in cache");
  m_pickup_block = m_real_cache->block_get();
  ER_ASSERT(m_pickup_block, "FATAL: No block in non-empty cache");
}

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cached_block_pickup::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void cached_block_pickup::visit(representation::cell2D& cell) {
  ER_ASSERT(0 != cell.loc().first && 0 != cell.loc().second,
            "FATAL: Cell does not have coordinates");
  ER_ASSERT(cell.state_has_cache(), "FATAL: cell does not have cache");
  if (nullptr != m_orphan_block) {
    cell.entity(m_orphan_block);
    ER_DIAG("Cell gets orphan block%d", m_orphan_block->id());
  }
  cell.fsm().accept(*this);
} /* visit() */

void cached_block_pickup::visit(representation::arena_cache& cache) {
  cache.block_remove(m_pickup_block);
  cache.has_block_pickup();
} /* visit() */

void cached_block_pickup::visit(representation::arena_map& map) {
  ER_ASSERT(m_real_cache->n_blocks() >= 2, "FATAL: < 2 blocks in cache");
  int cache_id = m_real_cache->id();
  ER_ASSERT(-1 != cache_id, "FATAL: Cache ID undefined on block pickup");

  rcppsw::math::dcoord2 coord = m_real_cache->discrete_loc();
  ER_ASSERT(coord == rcppsw::math::dcoord2(cell_op::x(), cell_op::y()),
            "FATAL: Coordinates for cache%d (%zu, %zu)/cell(%zu, %zu) do not "
            "agree",
            cache_id,
            coord.first,
            coord.second,
            cell_op::y(),
            cell_op::y());

  representation::cell2D& cell = map.access(cell_op::x(), cell_op::y());
  ER_ASSERT(m_real_cache->n_blocks() == cell.block_count(),
            "FATAL: Cache/cell disagree on # of blocks: cache=%u/cell=%zu",
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
              "FATAL: cell@(%zu, %zu) with >= %u blocks does not have cache",
              cell_op::x(),
              cell_op::y(),
              base_cache::kMinBlocks);
  } else {
    m_real_cache->accept(*this);
    m_orphan_block = m_real_cache->block_get();
    cell.accept(*this);

    ER_ASSERT(cell.state_has_block(),
              "FATAL: cell@(%zu, %zu) with 1 block has cache",
              cell_op::x(),
              cell_op::y());

    map.cache_remove(m_real_cache);
    map.cache_removed(true);
  }
  m_pickup_block->accept(*this);
  ER_NOM("arena_map: fb%u: block%d from cache%d @(%zu, %zu) [%u blocks remain]",
         m_robot_index,
         m_pickup_block->id(),
         cache_id,
         cell_op::x(),
         cell_op::y(),
         (m_real_cache) ? m_real_cache->n_blocks() : 1);
} /* visit() */

void cached_block_pickup::visit(representation::perceived_arena_map& map) {
  representation::cell2D& cell =
      map.access<occupancy_grid::kCellLayer>(cell_op::x(), cell_op::y());
  ER_ASSERT(cell.state_has_cache(), "FATAL: Cell does not contain cache");
  ER_ASSERT(cell.cache()->n_blocks() == cell.block_count(),
            "FATAL: perceived cache/cell disagree on # of blocks: "
            "cache=%u/cell/%zu",
            cell.cache()->n_blocks(),
            cell.block_count());

  ER_ASSERT(cell.cache()->contains_block(m_pickup_block),
            "FATAL: perceived cache does not contain ref to block to be picked "
            "up");

  if (cell.cache()->n_blocks() > 2) {
    cell.cache()->block_remove(m_pickup_block);
    cell.accept(*this);
    ER_ASSERT(cell.state_has_cache(),
              "FATAL: cell@(%zu, %zu) with >= 2 blocks does not have cache",
              cell_op::x(),
              cell_op::y());
    ER_NOM(
        "perceived_arena_map: fb%u: block%d from cache%d@(%zu, %zu) [%u "
        "blocks remain]",
        m_robot_index,
        m_pickup_block->id(),
        cell.cache()->id(),
        cell_op::x(),
        cell_op::y(),
        cell.cache()->n_blocks());

  } else {
    int id = cell.cache()->id();
    cell.cache()->block_remove(m_pickup_block);
    map.cache_remove(cell.cache());
    ER_NOM(
        "perceived_arena_map: fb%u: block%d from cache%d@(%zu, %zu) [cache "
        "empty]",
        m_robot_index,
        m_pickup_block->id(),
        id,
        cell_op::x(),
        cell_op::y());
  }
} /* visit() */

void cached_block_pickup::visit(representation::block& block) {
  ER_ASSERT(-1 != block.id(), "FATAL: Unamed block");
  block.add_carry();
  block.robot_index(m_robot_index);

  /* Move block out of sight */
  block.move_out_of_sight();
  ER_NOM("block: block%d is now carried by fb%u", block.id(), m_robot_index);
} /* visit() */

void cached_block_pickup::visit(
    controller::depth1::foraging_controller& controller) {
  controller.perception()->map()->accept(*this);
  controller.block(m_pickup_block);
  controller.current_task()->accept(*this);

  ER_NOM("depth1_foraging_controller: %s picked up block%d",
         controller.GetId().c_str(),
         m_pickup_block->id());
} /* visit() */

void cached_block_pickup::visit(tasks::collector& task) {
  static_cast<fsm::block_to_nest_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void cached_block_pickup::visit(fsm::block_to_nest_fsm& fsm) {
  fsm.inject_event(controller::foraging_signal::BLOCK_PICKUP,
                   state_machine::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);
