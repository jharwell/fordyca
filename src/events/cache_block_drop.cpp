/**
 * @file cache_block_drop.cpp
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
#include "fordyca/events/cache_block_drop.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/fsm/depth1/block_to_cache_fsm.hpp"
#include "fordyca/fsm/block_to_nest_fsm.hpp"
#include "fordyca/tasks/foraging_task.hpp"
#include "fordyca/tasks/forager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_block_drop::cache_block_drop(
    const std::shared_ptr<rcppsw::er::server>& server,
    representation::block* block, representation::cache* cache) :
    cell_op(cache->discrete_loc().first, cache->discrete_loc().second),
    client(server),
    m_block(block),
    m_cache(cache) {
  client::insmod("cache_block_drop",
                    rcppsw::er::er_lvl::DIAG,
                    rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Depth1 Foraging
 ******************************************************************************/
void cache_block_drop::visit(representation::perceived_cell2D& cell) {
  cell.cell().accept(*this);
} /* visit() */

void cache_block_drop::visit(representation::cell2D& cell) {
  cell.fsm().accept(*this);
} /* visit() */

void cache_block_drop::visit(fsm::cell2D_fsm& fsm) {
  ER_ASSERT(fsm.state_has_cache(), "FATAL: cell does not contain a cache");
  fsm.event_block_drop();
} /* visit() */

void cache_block_drop::visit(representation::arena_map& map) {
  ER_ASSERT(-1 != m_block->robot_index(), "FATAL: undefined robot index");
  int index = m_block->robot_index();
  m_block->accept(*this);
  m_cache->accept(*this);
  map.access(cell_op::x(), cell_op::y()).accept(*this);
  ER_NOM("arena_map: fb%d dropped block%d in cache%d (%zu blocks total)",
         index, m_block->id(), m_cache->id(), m_cache->n_blocks());
} /* visit() */

void cache_block_drop::visit(representation::perceived_arena_map& map) {
  map.access(cell_op::x(), cell_op::y()).accept(*this);
} /* visit() */

void cache_block_drop::visit(representation::block& block) {
  block.robot_index(-1);
} /* visit() */

void cache_block_drop::visit(representation::cache& cache) {
  cache.block_add(m_block);
  cache.inc_block_drops();
} /* visit() */

void cache_block_drop::visit(controller::depth1::foraging_controller& controller) {
  controller.block(nullptr);
  controller.map()->accept(*this);
  controller.current_task()->accept(*this);

  ER_NOM("depth1_foraging_controller: %s dropped block%d in cache%d",
         controller.GetId().c_str(), m_block->id(), m_cache->id());
} /* visit() */

void cache_block_drop::visit(tasks::forager& task) {
  static_cast<fsm::depth1::block_to_cache_fsm*>(task.mechanism())->accept(*this);
} /* visit() */

void cache_block_drop::visit(fsm::depth1::block_to_cache_fsm& fsm) {
  ER_NOM("block_to_cache_fsm: register cache_block_drop event");
  fsm.inject_event(controller::foraging_signal::BLOCK_DROP,
                   state_machine::event_type::NORMAL);
} /* visit() */

NS_END(events, fordyca);
