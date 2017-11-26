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
#include "rcppsw/er/client.hpp"
#include "fordyca/events/cell_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation {
class perceived_arena_map;
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
class cache_block_drop : public cell_op,
                         public rcppsw::er::client,
                         public visitor::visit_set<controller::depth1::foraging_controller,
                                                   fsm::depth1::block_to_cache_fsm,
                                                   tasks::forager,
                                                   representation::perceived_arena_map,
                                                   representation::block,
                                                   representation::cache,
                                                   representation::arena_map> {
 public:
  cache_block_drop(const std::shared_ptr<rcppsw::er::server>& server,
                   representation::block* block, representation::cache* cache,
                   double resolution);
  ~cache_block_drop(void) { client::rmmod(); }

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
  void visit(fsm::cell2D_fsm& fsm) override;

  /**
   * @brief Update the arena_map on a block drop by distributing the block in a
   * new location and updating the block so that it no longer thinks it is
   * carried by a robot.
   *
   * @param map The map to update (there is only ever one...)
   */
  void visit(representation::arena_map& map) override;
  void visit(representation::perceived_arena_map& map) override;

  void visit(representation::block& block) override;
  void visit(representation::cache& cache) override;
  void visit(class representation::perceived_cell2D& cell) override;
  void visit(controller::depth1::foraging_controller& controller) override;
  void visit(fsm::depth1::block_to_cache_fsm& fsm) override;
  void visit(tasks::forager& task) override;

 private:
  cache_block_drop(const cache_block_drop& op) = delete;
  cache_block_drop& operator=(const cache_block_drop& op) = delete;

  double m_resolution;
  representation::block* m_block;
  representation::cache* m_cache;
  std::shared_ptr<rcppsw::er::server> m_server;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_BLOCK_DROP_HPP_ */
