/**
 * @file likelihood_cache_search.cpp
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
#include "fordyca/fsm/expstrat/likelihood_cache_search.hpp"

#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/tasks/argument.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void likelihood_cache_search::task_start(const rta::taskable_argument*) {
  if (auto loc = mc_store->last_cache_loc()) {
    tasks::vector_argument v(kCACHE_ARRIVAL_TOL, *loc);
    localized_search::task_start(&v);
  } else {
    localized_search::task_start(nullptr);
  }
} /* task_start() */

NS_END(expstrat, fsm, fordyca);
