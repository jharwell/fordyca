/**
 * \file ledtaxis_cache_search.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/explore/ledtaxis_cache_search.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
ledtaxis_cache_search::ledtaxis_cache_search(
    const fstrategy::strategy_params* params,
    rmath::rng* rng)
    : foraging_strategy(params, rng),
      ER_CLIENT_INIT("fordyca.fsm.strategy.ledtaxis_cache_search"),
      m_crw(params, rng),
      m_taxis(params, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ledtaxis_cache_search::task_execute(void) {
  /*
   * If we are in a cache and executing this explore behavior, then the cache we
   * are currently in must be unsuitable for some reason. Don't try to taxis to
   * a cache signal in this cache, as it will not serve any purpose. Instead,
   * wander until the cache becomes suitable or something else changes.
   */
  if (saa()->sensing()->env()->detect("cache")) {
    if (m_taxis.task_running()) {
      m_taxis.task_reset();
    }
  } else {
    /* Otherwise, we are not inside a cache, so taxis to one */
    if (m_taxis.task_running()) {
      m_taxis.task_execute();
      return;
    }
  }
  /*
   * If we get here, then we are probably inside an unsuitable cache, so just
   * fall back to wandering.
   */
  if (!m_crw.task_running()) {
    m_crw.task_reset();
    m_crw.task_start(nullptr);
  }
  m_crw.task_execute();
} /* task_execute() */

void ledtaxis_cache_search::task_start(cta::taskable_argument*) {
  saa()->sensing()->blobs()->enable();
  m_taxis.task_start(nullptr);
} /* task_start() */

void ledtaxis_cache_search::task_reset(void) {
  saa()->sensing()->blobs()->disable();
  m_taxis.task_reset();
  m_crw.task_reset();
} /* task_reset() */

NS_END(explore, strategy, fordyca);
