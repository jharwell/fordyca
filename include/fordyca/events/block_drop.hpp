/**
 * @file block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_op.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace support {
class block_stat_collector;
} /* namespace support */

namespace representation {
class cell2D_fsm;
} /* namespace representation */

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_drop : public block_op, public rcppsw::common::er_client,
                   public visitor::can_visit<support::block_stat_collector, void>,
                   public visitor::can_visit<representation::cell2D_fsm, void> {
 public:
  block_drop(const std::shared_ptr<rcppsw::common::er_server>& server,
             representation::block* block);
  ~block_drop(void) { rmmod(); }

  /**
   * @brief Update a cell on a block drop.
   *
   * @param cell The cell to update.
   */
  void visit(class representation::cell2D& cell);

  /**
   * @brief Update the FSM associated with a cell on a block drop.
   *
   * @param fsm The FSM associated with the cell to update.
   */
  void visit(representation::cell2D_fsm& fsm);

  /**
   * @brief Update the arena_map on a block drop by distributing the block in a
   * new location and updating the block so that it no longer thinks it is
   * carried by a robot.
   *
   * @param map The map to update (there is only ever one...)
   */
  void visit(representation::arena_map& map);

  /**
   * @brief Update a block with the knowledge that it has been dropped.
   *
   * @param block The block to update.
   */
  void visit(representation::block& block);

  void visit(support::block_stat_collector& collector);

  /**
   * @brief Drop a carried block in the nest, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because dropping of blocks
   * needs to be done in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  void visit(controller::random_foraging_controller& controller);

  void visit(support::cache_update_handler& handler);

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  representation::block* block(void) const { return m_block; }

 private:
  block_drop(const block_drop& op) = delete;
  block_drop& operator=(const block_drop& op) = delete;
  representation::block* m_block;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_DROP_HPP_ */
