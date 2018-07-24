/**
 * @file new_cache_selector.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth2/new_cache_selector.hpp"
#include "fordyca/math/new_cache_utility.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/controller/cache_selection_matrix.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
new_cache_selector::new_cache_selector(
    std::shared_ptr<rcppsw::er::server> server,
    const controller::cache_selection_matrix* const csel_matrix)
    : client(server), mc_matrix(csel_matrix) {
  insmod("new_cache_selector", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::perceived_block new_cache_selector::calc_best(
    const std::list<representation::perceived_block>& new_caches,
    argos::CVector2 robot_loc) {
  representation::perceived_block best;
  ER_ASSERT(!new_caches.empty(), "FATAL: no known new caches");

  double max_utility = 0.0;
  for (auto& c : new_caches) {
    math::new_cache_utility u(c.ent->real_loc(),
                              boost::get<argos::CVector2>(
                                  mc_matrix->find("nest_center")->second));

    double utility = u.calc(robot_loc, c.density.last_result());
    ER_ASSERT(utility > 0.0, "FATAL: Bad utility calculation");
    ER_DIAG("Utility for new_cache%d loc=(%zu, %zu), density=%f: %f",
            c.ent->id(),
            c.ent->discrete_loc().first,
            c.ent->discrete_loc().second,
            c.density.last_result(),
            utility);

    if (utility > max_utility) {
      best = c;
      max_utility = utility;
    }
  } /* for(new_cache..) */

  ER_ASSERT(nullptr != best.ent, "FATAL: No best new cache found?");

  ER_NOM("Best utility: new_cache%d at (%u, %u): %f",
         best.ent->id(),
         best.ent->discrete_loc().first,
         best.ent->discrete_loc().second,
         max_utility);
  return best;
} /* calc_best() */

NS_END(depth2, controller, fordyca);
