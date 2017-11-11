/**
 * @file cache_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation {
class cell2D;
class perceived_cell2D;
class cell2D_fsm;
class cache;
class block;
class arena_map;
};

namespace fsm { namespace depth1 { class block_to_cache_fsm; }}
namespace controller { namespace depth1 { class foraging_controller; }}
namespace tasks { class forager; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cache_block_drop : public visitor::visitor,
                         public rcppsw::common::er_client,
                         public visitor::visit_set<controller::depth1::foraging_controller,
                                                   fsm::depth1::block_to_cache_fsm,
                                                   tasks::forager,
                                                   representation::cell2D,
                                                   representation::cell2D_fsm,
                                                   representation::perceived_cell2D,
                                                   representation::block,
                                                   representation::arena_map> {
 public:
  cache_block_drop(const std::shared_ptr<rcppsw::common::er_server>& server,
                   representation::block* block, representation::cache* cache);
  ~cache_block_drop(void) { er_client::rmmod(); }

  /* depth1 foraging */
  /**
   * @brief Update a cell on a block drop.
   *
   * @param cell The cell to update.
   */
  void visit(class representation::cell2D& cell) override;

  /**
   * @brief Update the FSM associated with a cell on a block drop.
   *
   * @param fsm The FSM associated with the cell to update.
   */
  void visit(representation::cell2D_fsm& fsm) override;

  /**
   * @brief Update the arena_map on a block drop by distributing the block in a
   * new location and updating the block so that it no longer thinks it is
   * carried by a robot.
   *
   * @param map The map to update (there is only ever one...)
   */
  void visit(representation::arena_map& map) override;

  /**
   * @brief Update a block with the knowledge that it has been dropped.
   *
   * @param block The block to update.
   */
  void visit(representation::block& block) override;
  void visit(class representation::perceived_cell2D& cell) override;
  void visit(controller::depth1::foraging_controller& controller) override;
  void visit(fsm::depth1::block_to_cache_fsm& fsm) override;
  void visit(tasks::forager& task) override;

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  representation::block* block(void) const { return m_block; }

 private:
  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;
  representation::block* m_block;
  representation::cache* m_cache;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_ */
