/**
 * @file base_block.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);

/*******************************************************************************
 *  Static Variables
 ******************************************************************************/
const rmath::vector2u base_block::kOutOfSightDLoc = rmath::vector2u(1000, 1000);
const rmath::vector2d base_block::kOutOfSightRLoc =
    rmath::vector2d(1000.0, 1000.0);

/******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_block::move_out_of_sight(void) {
  rloc(kOutOfSightRLoc);
  dloc(kOutOfSightDLoc);
} /* move_out_of_sight() */

void base_block::reset_metrics(void) {
  m_transporters = 0;
  m_first_pickup_time = rtypes::timestep(0);
  m_first_pickup = false;
  m_dist_time = rtypes::timestep(0);
  m_nest_drop_time = rtypes::timestep(0);
} /* reset_metrics(); */

void base_block::first_pickup_time(rtypes::timestep time) {
  if (!m_first_pickup) {
    m_first_pickup_time = time;
    m_first_pickup = true;
  }
} /* first_pickup_time() */

 rtypes::timestep base_block::total_transport_time(void) const {
  return m_nest_drop_time - m_first_pickup_time;
} /* total_transport_time() */

 rtypes::timestep base_block::initial_wait_time(void) const {
  return m_first_pickup_time - m_dist_time;
} /* initial_wait_time() */

NS_END(repr, fordyca);
