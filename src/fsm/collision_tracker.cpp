/**
 * @file collision_tracker.cpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/fsm/collision_tracker.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool collision_tracker::in_collision_avoidance(void) const {
  return m_in_avoidance;
} /* in_collision_avoidance() */

bool collision_tracker::entered_collision_avoidance(void) const {
  return m_entered_avoidance;
} /* entered_collision_avoidance() */

bool collision_tracker::exited_collision_avoidance(void) const {
  return m_exited_avoidance;
} /* exited_collision_avoidance() */

rtypes::timestep collision_tracker::collision_avoidance_duration(void) const {
  if (m_exited_avoidance) {
    return mc_saa->sensing()->tick() - m_avoidance_start;
  }
  return rtypes::timestep(0);
} /* collision_avoidance_duration() */

rmath::vector2u collision_tracker::avoidance_loc(void) const {
  return mc_saa->sensing()->discrete_position();
} /* avoidance_loc() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void collision_tracker::ca_enter(void) {
  if (!m_in_avoidance) {
    if (!m_entered_avoidance) {
      m_entered_avoidance = true;
      m_avoidance_start = mc_saa->sensing()->tick();
    }
  } else {
    m_entered_avoidance = false;
  }
  m_in_avoidance = true;
} /* ca_enter() */

void collision_tracker::ca_exit(void) {
  if (!m_exited_avoidance) {
    if (m_in_avoidance) {
      m_exited_avoidance = true;
    }
  } else {
    m_exited_avoidance = false;
  }
  m_in_avoidance = false;
  m_entered_avoidance = false; /* catches 1 timestep avoidances correctly */
} /* ca_exit() */

NS_END(fsm, fordyca);
