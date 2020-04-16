/**
 * \file localized_search.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/fsm/expstrat/localized_search.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void localized_search::task_execute(void) {
  if (m_vfsm.task_running()) {
    m_vfsm.task_execute();
    if (m_vfsm.task_finished()) {
      m_crw.task_start(nullptr);
    }
  } else { /* CRW explore behavior */
    m_crw.task_execute();
  }
} /* task_execute() */

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool localized_search::in_collision_avoidance(void) const {
  return (m_vfsm.task_running() && m_vfsm.in_collision_avoidance()) ||
         (m_crw.task_running() && m_crw.in_collision_avoidance());
} /* in_collision_avoidance() */

bool localized_search::entered_collision_avoidance(void) const {
  return (m_vfsm.task_running() && m_vfsm.entered_collision_avoidance()) ||
         (m_crw.task_running() && m_crw.entered_collision_avoidance());
} /* entered_collision_avoidance() */

bool localized_search::exited_collision_avoidance(void) const {
  return (m_vfsm.task_running() && m_vfsm.exited_collision_avoidance()) ||
         (m_crw.task_running() && m_crw.exited_collision_avoidance());
} /* exited_collision_avoidance() */

rtypes::timestep localized_search::collision_avoidance_duration(void) const {
  if (m_vfsm.task_running()) {
    return m_vfsm.collision_avoidance_duration();
  } else if (m_crw.task_running()) {
    return m_crw.collision_avoidance_duration();
  } else {
    return rtypes::timestep(0);
  }
} /* collision_avoidance_duration() */

rmath::vector2z localized_search::avoidance_loc(void) const {
  ER_ASSERT(m_vfsm.task_running() || m_crw.task_running(),
            "In collision avoidance without running task?");
  if (m_vfsm.task_running()) {
    return m_vfsm.avoidance_loc();
  } else {
    return m_crw.avoidance_loc();
  }
} /* collision_avoidance_duration() */

NS_END(expstrat, fsm, fordyca);
