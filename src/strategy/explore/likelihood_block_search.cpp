/**
 * \file likelihood_block_search.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/strategy/explore/likelihood_block_search.hpp"

#include "cosm/spatial/fsm/point_argument.hpp"

#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/

likelihood_block_search::likelihood_block_search(
    const fstrategy::strategy_params* params,
    rmath::rng* rng)
    : localized_search(params, rng) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void likelihood_block_search::task_start(cta::taskable_argument*) {
  if (auto loc = accessor()->last_block_loc()) {
    csfsm::point_argument v(fsm::kBLOCK_ARRIVAL_TOL, *loc);
    localized_search::task_start(&v);
  } else {
    localized_search::task_start(nullptr);
  }
} /* task_start() */

NS_END(explore, strategy, fordyca);
