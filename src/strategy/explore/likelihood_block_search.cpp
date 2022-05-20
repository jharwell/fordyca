/**
 * \file likelihood_block_search.cpp
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
