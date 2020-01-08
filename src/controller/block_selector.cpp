/**
 * \file block_selector.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/repr/base_block2D.hpp"

#include "fordyca/math/block_utility.hpp"

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
boost::optional<ds::dp_block_map::value_type> block_selector::operator()(
    const ds::dp_block_map& blocks,
    const rmath::vector2d& position) {
  double max_utility = 0.0;
  ds::dp_block_map::value_type best{nullptr, {}};

  ER_ASSERT(!blocks.empty(), "No known perceived blocks");
  for (const auto& b : blocks.const_values_range()) {
    if (block_is_excluded(position, b.ent())) {
      continue;
    }

    /*
     * Only two options for right now: cube blocks or ramp blocks. This will
     * undoubtedly have to change in the future.
     */
    double priority =
        (crepr::block_type::ekCUBE == b.ent()->type())
            ? boost::get<double>(mc_matrix->find(bselm::kCubePriority)->second)
            : boost::get<double>(mc_matrix->find(bselm::kRampPriority)->second);
    rmath::vector2d nest_loc =
        boost::get<rmath::vector2d>(mc_matrix->find(bselm::kNestLoc)->second);

    double utility =
        math::block_utility(b.ent()->rloc(),
                            nest_loc)(position, b.density(), priority);

    ER_DEBUG("Utility for block%d@%s/%s, density=%f: %f",
             b.ent()->id().v(),
             b.ent()->rloc().to_str().c_str(),
             b.ent()->dloc().to_str().c_str(),
             b.density().v(),
             utility);
    if (utility > max_utility) {
      best = b;
      max_utility = utility;
    }
  } /* for(block..) */

  if (nullptr != best.ent()) {
    ER_INFO("Best utility: block%d@%s/%s: %f",
            best.ent()->id().v(),
            best.ent()->rloc().to_str().c_str(),
            best.ent()->dloc().to_str().c_str(),
            max_utility);
    return boost::make_optional(best);
  } else {
    ER_WARN("No best block found: all known blocks excluded!");
    return boost::optional<ds::dp_block_map::value_type>();
  }
} /* operator() */

bool block_selector::block_is_excluded(
    const rmath::vector2d& position,
    const crepr::base_block2D* const block) const {
  double block_dim = std::min(block->xspan().span(), block->yspan().span());
  if ((position - block->rloc()).length() <= block_dim) {
    ER_DEBUG("Ignoring block%d@%s/%s: Too close (%f <= %f)",
             block->id().v(),
             block->rloc().to_str().c_str(),
             block->dloc().to_str().c_str(),
             (position - block->rloc()).length(),
             block_dim);
    return true;
  }
  auto exceptions = boost::get<std::vector<rtypes::type_uuid>>(
      mc_matrix->find(bselm::kSelExceptions)->second);
  if (std::any_of(exceptions.begin(), exceptions.end(), [&](auto& id) {
        return id == block->id();
      })) {
    ER_DEBUG("Ignoring block%d@%s/%s: On exception list",
             block->id().v(),
             block->rloc().to_str().c_str(),
             block->dloc().to_str().c_str());
    return true;
  }
  return false;
} /* block_is_excluded() */

NS_END(controller, fordyca);
