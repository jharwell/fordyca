/**
 * @file cell_perception.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CELL_PERCEPTION_HPP_
#define INCLUDE_FORDYCA_EVENTS_CELL_PERCEPTION_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/common/er_server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation {
class perceived_cell2D;
class cell_entity;
} /* namespace representation */

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cell_perception : public visitor::visitor,
                        public visitor::can_visit<representation::perceived_cell2D,
                                                  void> {
 public:
  explicit cell_perception(const std::shared_ptr<rcppsw::common::er_server>& server,
                           uint8_t cell_state,
                           representation::cell_entity* entity = nullptr);

  /**
   * @brief A robot has encountered this cell during exploring or travel
   * (i.e. the cell fell within its LOS). Each timestep that the cell remains in
   * the robot's LOS, it is encountered again. Each encounter causes a unit
   * amout of pheromone to be deposited on the cell; the information relevance
   * is reinforced.
   *
   * This is OK, because blocks CAN suddenly disappear from a robot's LOS if it
   * is picked up by another robot (robots are generally unaware of each
   * other).
   */
  void visit(representation::perceived_cell2D& cell);

 private:
  cell_perception(const cell_perception& op) = delete;
  cell_perception& operator=(const cell_perception& op) = delete;

  uint8_t m_cell_state;
  representation::cell_entity* m_entity;
  std::shared_ptr<rcppsw::common::er_server> m_server;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CELL_PERCEPTION_HPP_ */
