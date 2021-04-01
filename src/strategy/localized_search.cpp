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
#include "fordyca/strategy/explore/localized_search.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
localized_search::localized_search(csubsystem::saa_subsystemQ3D* saa,
                                   rmath::rng* rng)
    : foraging_strategy(saa, rng),
      ER_CLIENT_INIT("fordyca.fsm.strategy.localized_search"),
      m_vfsm(saa, rng),
      m_crw(saa, rng) {}

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
 * Interference Metrics
 ******************************************************************************/
bool localized_search::exp_interference(void) const {
  return (m_vfsm.task_running() && m_vfsm.exp_interference()) ||
         (m_crw.task_running() && m_crw.exp_interference());
} /* exp_interference() */

bool localized_search::entered_interference(void) const {
  return (m_vfsm.task_running() && m_vfsm.entered_interference()) ||
         (m_crw.task_running() && m_crw.entered_interference());
} /* entered_interference() */

bool localized_search::exited_interference(void) const {
  return (m_vfsm.task_running() && m_vfsm.exited_interference()) ||
         (m_crw.task_running() && m_crw.exited_interference());
} /* exited_interference() */

rtypes::timestep localized_search::interference_duration(void) const {
  if (m_vfsm.task_running()) {
    return m_vfsm.interference_duration();
  } else if (m_crw.task_running()) {
    return m_crw.interference_duration();
  } else {
    return rtypes::timestep(0);
  }
} /* interference_duration() */

rmath::vector3z localized_search::interference_loc3D(void) const {
  ER_ASSERT(m_vfsm.task_running() || m_crw.task_running(),
            "In collision interference without running task?");
  if (m_vfsm.task_running()) {
    return m_vfsm.interference_loc3D();
  } else {
    return m_crw.interference_loc3D();
  }
} /* interference_loc3D() */

NS_END(explore, strategy, fordyca);
