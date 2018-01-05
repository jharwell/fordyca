/**
 * @file block_selector.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth0/block_selector.hpp"
#include "fordyca/expressions/block_utility.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_selector::block_selector(const std::shared_ptr<rcppsw::er::server> &server,
                               argos::CVector2 nest_loc)
    : client(server), m_nest_loc(nest_loc) {
  insmod("block_selector", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::const_perceived_block block_selector::calc_best(
    const std::list<representation::const_perceived_block>& blocks,
    argos::CVector2 robot_loc) {
  double max_utility = 0.0;
  const representation::block *best = nullptr;

  ER_ASSERT(!blocks.empty(), "FATAL: no known perceived blocks");
  for (auto pair : blocks) {
    expressions::block_utility u(pair.first->real_loc(), m_nest_loc);

    double utility = u.calc(robot_loc, pair.second);
    ER_DIAG("Utility for block%d loc=(%zu, %zu), density=%f: %f",
            pair.first->id(),
            pair.first->discrete_loc().first,
            pair.first->discrete_loc().second,
            pair.second,
            utility);
    if (utility > max_utility) {
      max_utility = utility;
      best = pair.first;
    }
  } /* for(block..) */

  ER_ASSERT(nullptr != best, "FATAL: No best perceived block?");
  ER_NOM("Best utility: block%d at (%zu, %zu): %f",
         best->id(),
         best->discrete_loc().first,
         best->discrete_loc().second,
         max_utility);
  return representation::const_perceived_block(best, max_utility);
} /* calc_best() */

NS_END(depth0, controller, fordyca);
