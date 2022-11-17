/**
 * \file localized_search.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
