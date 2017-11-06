/**
 * @file block_found.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/perceived_cell_op.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation { class block; }
namespace tasks { class generalist; class forager; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_found
 *
 * @brief Event that is created whenever a block is found, via floor sensor
 * detection, or via a block appearing in a robot's LOS. These events are not
 * processed by the \ref arena_map, and exist only in a robot's perception.
 */
class block_found : public perceived_cell_op,
                    public rcppsw::common::er_client,
                    public visitor::visit_set<tasks::generalist,
                                              tasks::forager> {
 public:
  block_found(const std::shared_ptr<rcppsw::common::er_server>& server,
              const representation::block* block, size_t x, size_t y);
  ~block_found(void) { er_client::rmmod(); }

  void visit(representation::cell2D& cell) override;
  void visit(representation::perceived_cell2D& cell) override;
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
  void visit(representation::perceived_arena_map& map) override;

  /**
   * @brief Drop a carried block in the nest, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because dropping of blocks
   * needs to be done in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  void visit(controller::memory_foraging_controller& controller) override;

  void visit(controller::depth1_foraging_controller& controller) override;
  void visit(tasks::generalist& task) override;
  void visit(tasks::forager& task) override;

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  const representation::block* block(void) const { return m_block; }

 private:
  block_found(const block_found& op) = delete;
  block_found& operator=(const block_found& op) = delete;
  const representation::block* m_block;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_FOUND_HPP_ */
