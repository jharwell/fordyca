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
localized_search::localized_search(const fstrategy::strategy_params* params,
                                   rmath::rng* rng)
    : foraging_strategy(params, rng),
      ER_CLIENT_INIT("fordyca.fsm.strategy.localized_search"),
      m_vfsm(params->fsm, rng),
      m_crw(params, rng) {}

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

NS_END(explore, strategy, fordyca);
