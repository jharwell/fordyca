/**
 * \file utility_cache_search.cpp
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
#include "fordyca/strategy/explore/utility_cache_search.hpp"

#include <numeric>

#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/fsm/point_argument.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "fordyca/subsystem/perception/ds/dpo_store.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/d2/cache_site_selector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, strategy, explore);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
utility_cache_search::utility_cache_search(
    const fstrategy::strategy_params* params,
    rmath::rng* rng)
    : localized_search(params, rng),
      mc_matrix(params->csel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void utility_cache_search::task_start(cta::taskable_argument*) {
  auto range = accessor()->known_blocks();
  rmath::vector2d position;
  if (!range.empty()) {
    position = std::accumulate(range.begin(),
                               range.end(),
                               rmath::vector2d(),
                               [&](rmath::vector2d& sum, const auto& bent) {
                                 return sum + bent->rcenter2D();
                               }) /
               range.size();
  } else {
    position = saa()->sensing()->rpos2D();
  }
  fsm::d2::cache_site_selector sel(mc_matrix);
  if (auto site = sel(accessor()->known_caches(), position, rng())) {
    csfsm::point_argument v(fsm::kCACHE_ARRIVAL_TOL, *site);
    localized_search::task_start(&v);
  } else {
    localized_search::task_start(nullptr);
  }
} /* task_start() */

NS_END(explore, strategy, fordyca);
