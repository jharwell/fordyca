/**
 * @file block_acquisition_validator.cpp
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
#include "fordyca/fsm/block_acquisition_validator.hpp"
#include <numeric>

#include "fordyca/controller/block_sel_matrix.hpp"
#include "fordyca/ds/dp_block_map.hpp"
#include "fordyca/repr/base_block.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);
using bselm = controller::block_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
block_acquisition_validator::block_acquisition_validator(
    const ds::dp_block_map* map,
    const controller::block_sel_matrix* matrix)
    : ER_CLIENT_INIT("fordyca.fsm.block_acquisition_validator"),
      mc_map(map),
      mc_matrix(matrix) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
__rcsw_pure bool block_acquisition_validator::operator()(
    const rmath::vector2d& loc,
    uint id) const {
  auto block = mc_map->find(id);

  /* Sanity checks for acqusition */
  if (nullptr == block) {
    ER_WARN("Acquisition of free block%d@%s invalid: no such block",
            id,
            loc.to_str().c_str());
    return false;
  }
  auto& config = boost::get<config::block_sel::pickup_policy_config>(
      mc_matrix->find(bselm::kPickupPolicy)->second);

  /*
   * Unless we have the cluster proximity policy, we are good to go on
   * validation if we make it this far.
   */
  if (bselm::kPickupPolicyClusterProx == config.policy) {
    auto range = mc_map->const_values_range();
    if (!range.empty()) {
      auto avg_position =
          std::accumulate(range.begin(),
                          range.end(),
                          rmath::vector2d(),
                          [&](rmath::vector2d& sum, const auto& bent) {
                            return sum + bent.ent()->real_loc();
                          }) /
          boost::size(range);

      return (loc - avg_position).length() < config.prox_dist;
    }
  } /* for(&bent..) */
  return true;
} /* operator()() */

NS_END(fsm, fordyca);
