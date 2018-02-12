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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/***********************e*******************************************************
 * Member Functions
 ******************************************************************************/
void block::move_out_of_sight(void) {
  real_loc(argos::CVector2(100.0, 100.0));
  discrete_loc(rcppsw::math::dcoord2(100, 100));
} /* move_out_of_sight() */

std::unique_ptr<block> block::clone(void) const {
  std::unique_ptr<block> tmp = rcppsw::make_unique<block>(cell_entity::xsize());
  tmp->discrete_loc(this->discrete_loc());
  tmp->real_loc(this->real_loc());
  tmp->id(this->id());
  tmp->reset_index();
  return tmp;
} /* clone() */
NS_END(representation, fordyca);
