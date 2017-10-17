/**
 * @file perceived_arena_op.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_PERCEIVED_ARENA_OP_HPP_
#define INCLUDE_FORDYCA_EVENTS_PERCEIVED_ARENA_OP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;

namespace controller {
class memory_foraging_controller;
} /* namespace controller */

namespace representation {
class cell2D;
class perceived_cell2D;
class perceived_arena_map;
class cell2D_fsm;
} /* namespace representation */

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class perceived_arena_op : public visitor::visitor,
                           public visitor::can_visit<controller::memory_foraging_controller>,
                           public visitor::can_visit<representation::cell2D>,
                           public visitor::can_visit<representation::perceived_cell2D>,
                           public visitor::can_visit<representation::cell2D_fsm>,
                           public visitor::can_visit<representation::perceived_arena_map> {
 public:
  perceived_arena_op(void) {}
  virtual ~perceived_arena_op(void) {}
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_PERCEIVED_ARENA_OP_HPP_ */