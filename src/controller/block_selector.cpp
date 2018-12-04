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
#include "fordyca/controller/block_selector.hpp"
#include "fordyca/math/block_utility.hpp"
#include "fordyca/representation/base_block.hpp"
#include "fordyca/representation/cube_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
using bselm = block_sel_matrix;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_selector::block_selector(const block_sel_matrix* const sel_matrix)
    : ER_CLIENT_INIT("fordyca.controller.depth0.block_selector"),
      mc_matrix(sel_matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
representation::perceived_block block_selector::calc_best(
    const ds::perceived_block_list& blocks,
    const rmath::vector2d& position) {
  double max_utility = 0.0;
  representation::perceived_block best{nullptr, {}};

  ER_ASSERT(!blocks.empty(), "No known perceived blocks");
  for (auto& b : blocks) {
    if (block_is_excluded(position, b.ent.get())) {
      continue;
    }

    /*
     * Only two options for right now: cube blocks or ramp blocks. This will
     * undoubtedly have to change in the future.
     */
    double priority =
        (dynamic_cast<representation::cube_block*>(b.ent.get()))
            ? boost::get<double>(mc_matrix->find(bselm::kCubePriority)->second)
            : boost::get<double>(mc_matrix->find(bselm::kRampPriority)->second);
    rmath::vector2d nest_loc =
        boost::get<rmath::vector2d>(mc_matrix->find(bselm::kNestLoc)->second);

    double utility = math::block_utility(b.ent->real_loc(), nest_loc)(
        position, b.density.last_result(), priority);

    ER_DEBUG("Utility for block%d@%s/%s, density=%f: %f",
             b.ent->id(),
             b.ent->real_loc().to_str().c_str(),
             b.ent->discrete_loc().to_str().c_str(),
             b.density.last_result(),
             utility);
    if (utility > max_utility) {
      best = b;
      max_utility = utility;
    }
  } /* for(block..) */

  if (nullptr != best.ent) {
    ER_INFO("Best utility: block%d@%s/%s: %f",
            best.ent->id(),
            best.ent->real_loc().to_str().c_str(),
            best.ent->real_loc().to_str().c_str(),
            max_utility);
  } else {
    ER_WARN("No best block found: all known blocks excluded!");
  }
  return best;
} /* calc_best() */

bool block_selector::block_is_excluded(
    const rmath::vector2d& position,
    const representation::base_block* const block) const {
  double block_dim = std::min(block->xspan(block->real_loc()).span(),
                              block->yspan(block->real_loc()).span());
  if ((position - block->real_loc()).length() <= block_dim) {
    ER_DEBUG("Ignoring block%d@%s/%s: Too close (%f <= %f)",
             block->id(),
             block->real_loc().to_str().c_str(),
             block->discrete_loc().to_str().c_str(),
             (position - block->real_loc()).length(),
             block_dim);
    return true;
  }
  std::vector<int> exceptions = boost::get<std::vector<int>>(
      mc_matrix->find(bselm::kSelExceptions)->second);
  if (std::any_of(exceptions.begin(), exceptions.end(), [&](int id) {
        return id == block->id();
      })) {
    ER_DEBUG("Ignoring block%d@%s/%s: On exception list",
             block->id(),
             block->real_loc().to_str().c_str(),
             block->discrete_loc().to_str().c_str());
    return true;
  }
  return false;
} /* block_is_excluded() */

NS_END(controller, fordyca);
