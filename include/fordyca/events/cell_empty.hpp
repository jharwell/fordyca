/**
 * @file cell_empty.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL_EMPTY_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL_EMPTY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/cell_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation {
class arena_map;
class perceived_arena_map;
} /* namespace representation */

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cell_empty : public cell_op,
                   public visitor::can_visit<representation::arena_map>,
                   public visitor::can_visit<representation::perceived_arena_map> {
 public:
  cell_empty(size_t x, size_t y) : cell_op(x, y) {}

  /**
   * @brief Update a cell with the knowledge that it is now empty.
   *
   * @param cell The cell to update.
   */
  void visit(representation::cell2D& cell) override;

  /**
   * @brief Update a cell with the knowledge that it is now empty.
   *
   * @param cell The cell to update.
   */
  void visit(representation::perceived_cell2D& cell) override;

  /**
   * @brief Update the FSM associated with a cell with the knowledge that it is
   * now empty.
   *
   * @param fsm The FSM from the cell to update.
   */
  void visit(fsm::cell2D_fsm& fsm) override;

  void visit(representation::arena_map& map) override;
  void visit(representation::perceived_arena_map& map) override;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_EMPTY_HPP_ */
