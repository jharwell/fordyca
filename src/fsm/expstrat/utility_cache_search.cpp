/**
 * @file utility_cache_search.cpp
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
#include "fordyca/fsm/expstrat/utility_cache_search.hpp"
#include <numeric>

#include "fordyca/controller/depth2/cache_site_selector.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm, expstrat);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void utility_cache_search::task_start(const rta::taskable_argument*) {
  auto range = mc_store->blocks().const_values_range();
  rmath::vector2d position;
  if (!range.empty()) {
    position = std::accumulate(range.begin(),
                               range.end(),
                               rmath::vector2d(),
                               [&](rmath::vector2d& sum, const auto& bent) {
                                 return sum + bent.ent()->rloc();
                               }) /
               boost::size(range);
  } else {
    position = saa_subsystem()->sensing()->position();
  }
  controller::depth2::cache_site_selector sel(mc_matrix);
  if (auto site = sel(mc_store->caches(), mc_store->blocks(), position)) {
    tasks::vector_argument v(vector_fsm::kCACHE_ARRIVAL_TOL, *site);
    localized_search::task_start(&v);
  } else {
    localized_search::task_start(nullptr);
  }
} /* task_start() */

NS_END(expstrat, fsm, fordyca);
