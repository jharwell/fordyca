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
#include "fordyca/math/block_utility.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/cube_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_selector::block_selector(const block_selection_matrix* const sel_matrix)
    : ER_CLIENT_INIT("fordyca.controller.depth0.block_selector"),
      mc_matrix(sel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::perceived_block block_selector::calc_best(
    const perceived_block_list& blocks,
    argos::CVector2 robot_loc) {
  double max_utility = 0.0;
  representation::perceived_block best{nullptr, {}};

  ER_ASSERT(!blocks.empty(), "No known perceived blocks");
  for (auto& b : blocks) {
    if (block_is_excluded(robot_loc, b.ent.get())) {
      continue;
    }

    /*
     * Only two options for right now: cube blocks or ramp blocks. This will
     * undoubtedly have to change in the future.
     */
    double priority =
        (dynamic_cast<representation::cube_block*>(b.ent.get()))
            ? boost::get<double>(mc_matrix->find("cube_priority")->second)
            : boost::get<double>(mc_matrix->find("ramp_priority")->second);
    argos::CVector2 nest_loc =
        boost::get<argos::CVector2>(mc_matrix->find("nest_loc")->second);

    double utility = math::block_utility(b.ent->real_loc(), nest_loc)(
        robot_loc, b.density.last_result(), priority);

    ER_DEBUG("Utility for block%d loc=(%u, %u), density=%f: %f",
             b.ent->id(),
             b.ent->discrete_loc().first,
             b.ent->discrete_loc().second,
             b.density.last_result(),
             utility);
    if (utility > max_utility) {
      best = b;
      max_utility = utility;
    }
  } /* for(block..) */

  if (nullptr != best.ent) {
    ER_INFO("Best utility: block%d at (%f, %f) [%u, %u]: %f",
            best.ent->id(),
            best.ent->real_loc().GetX(),
            best.ent->real_loc().GetY(),
            best.ent->discrete_loc().first,
            best.ent->discrete_loc().second,
            max_utility);
  } else {
    ER_WARN("No best block found: all known blocks too close/on exception list!");
  }
  return best;
} /* calc_best() */

bool block_selector::block_is_excluded(
    const argos::CVector2& robot_loc,
    const representation::base_block* const block) const {
  if ((robot_loc - block->real_loc()).Length() <= kMinDist) {
    ER_INFO("Ignoring block%d@(%f,%f) [%u, %u]: Too close (%f < %f)",
             block->id(),
             block->real_loc().GetX(),
             block->real_loc().GetY(),
             block->discrete_loc().first,
             block->discrete_loc().second,
             (robot_loc - block->real_loc()).Length(),
             kMinDist);
    return true;
  }
  std::vector<int> exceptions =
      boost::get<std::vector<int>>(mc_matrix->find("sel_exceptions")->second);
  if (std::any_of(exceptions.begin(),
                  exceptions.end(),
                  [&](int id) { return id == block->id(); })) {
    ER_INFO("Ignoring block%d@(%f,%f): On exception list",
             block->id(),
             block->real_loc().GetX(),
             block->real_loc().GetY());
    return true;
  }
  return false;
} /* block_is_excluded() */

NS_END(depth0, controller, fordyca);
