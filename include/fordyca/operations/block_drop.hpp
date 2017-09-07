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

#ifndef INCLUDE_FORDYCA_OPERATIONS_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_OPERATIONS_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/operations/block_op.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace controller {
class random_foraging_controller;
class unpartitioned_task_controller;
} /* namespace controller */

namespace representation {
class cell2D_fsm;
class arena_map;
class block;
} /* namespace representation */

NS_START(operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_drop : public block_op, public rcppsw::common::er_client {
 public:
  block_drop(const std::shared_ptr<rcppsw::common::er_server>& server,
             representation::block* block);
  ~block_drop(void) { rmmod(); }

  void visit(representation::cell2D& cell);
  void visit(representation::cell2D_fsm& fsm);
  void visit(representation::arena_map& map);
  void visit(representation::block& block);

  /**
   * @brief Drop a carried block in the nest, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because dropping of blocks
   * needs to be done in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  void visit(controller::random_foraging_controller& controller);
  representation::block* block(void) const { return m_block; }

 private:
  block_drop(const block_drop& op) = delete;
  block_drop& operator=(const block_drop& op) = delete;
  representation::block* m_block;
};

NS_END(operations, fordyca);

#endif /* INCLUDE_FORDYCA_OPERATIONS_BLOCK_DROP_HPP_ */
