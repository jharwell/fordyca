/**
 * @file block.cpp
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
#include "fordyca/representation/block.hpp"
#include "fordyca/controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block::update_on_robot_pickup(size_t index) {
  ++m_carries;
  m_robot_index = index;

  /* Move block out of sight */
  set_loc(argos::CVector2(100.0, 100.0));
} /* update_on_robot_pickup() */

void block::update_on_arena_drop(const argos::CVector2& loc) {
  m_loc = loc;
  m_robot_index = -1;
} /* update_on_arena_drop() */

bool block::contains_point(const argos::CVector2& point) {
  double x = m_loc.GetX();
  double y = m_loc.GetY();
  if (point.GetX() < (x + (.5 * m_dimension)) &&
      point.GetX() > (x - (.5 * m_dimension)) &&
      point.GetY() < (y + (.5 * m_dimension)) &&
      point.GetY() > (y - (.5 * m_dimension))) {
    return true;
  }
  return false;
} /* contains_point() */

NS_END(representation, fordyca);
