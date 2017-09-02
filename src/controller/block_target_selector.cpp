/**
 * @file block_target_selector.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/block_target_selector.hpp"
#include "fordyca/expressions/expressions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_target_selector::block_target_selector(const std::shared_ptr<rcppsw::common::er_server>& server,
                                             argos::CVector2 nest_loc) :
    er_client(server),
    m_nest_loc(nest_loc) {
  insmod("block_target_selector",
          rcppsw::common::er_lvl::DIAG,
          rcppsw::common::er_lvl::NOM);
}


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::pair<const representation::block*, double> block_target_selector::calc_best(
    const std::list<std::pair<const representation::block*, double>> blocks,
    argos::CVector2 robot_loc) {
  double max_utility = 0.0;
  const representation::block* best;
  for (auto pair : blocks) {
    expressions::forage::block_utility u(pair.first->real_loc(), m_nest_loc);

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
  ER_NOM("Best utility: block%d at (%zu, %zu): %f",
         best->id(),
         best->discrete_loc().first,
         best->discrete_loc().second,
         max_utility);
  return std::pair<const representation::block*, double>(best, max_utility);
} /* calc_best() */

NS_END(controller, fordyca);
