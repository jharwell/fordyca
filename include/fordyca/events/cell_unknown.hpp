/**
 * @file cell_unknown.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL_UNKNOWN_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL_UNKNOWN_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace ds {
class cell2D;
class occupancy_grid;
} // namespace ds
namespace fsm {
class perceived_cell2D_fsm;
}

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cell_unknown
 * @ingroup events
 *
 * @brief Created whenever a cell within an occupancy grid needs to go into an
 * unknown state.
 *
 * This happens in two cases:
 *
 * 1. After its relevance expires.
 * 2. Before the robot sees it for the first time (ala Fog of War).
 */
class cell_unknown : public cell_op,
                     public visitor::can_visit<ds::occupancy_grid>,
                     public rcppsw::er::client<cell_unknown> {
 public:
  cell_unknown(uint x, uint y)
      : cell_op(x, y), ER_CLIENT_INIT("fordyca.ds.events.cell_unknown") {}

  /* stateful foraging */
  void visit(ds::cell2D& cell) override;
  void visit(ds::occupancy_grid& grid) override;
  void visit(fsm::cell2D_fsm& fsm) override;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_UNKNOWN_HPP_ */
