/**
 * @file cache_acquisition_validator.cpp
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
#include "fordyca/fsm/cache_acquisition_validator.hpp"
#include "fordyca/ds/dp_cache_map.hpp"
#include "fordyca/params/cache_sel/pickup_policy_params.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);
using cselm = controller::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_acquisition_validator::cache_acquisition_validator(
    const ds::dp_cache_map* map,
    const controller::cache_sel_matrix* csel_matrix)
    : ER_CLIENT_INIT("fordyca.fsm.cache_acquisition_validator"),
      mc_csel_matrix(csel_matrix),
      mc_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool cache_acquisition_validator::operator()(
    const rmath::vector2d& loc,
    int id,
    uint timestep) const {

  auto range = mc_map->values_range();
  auto it = std::find_if(range.begin(), range.end(),
                         [&](const auto& c) {
                           return c.ent()->id() == id; });

  if (range.end() == it) {
    ER_WARN("Cache%d near %s invalid for acquisition: no such cache",
            id,
            loc.to_str().c_str());
    return false;
  } else if (!it->ent()->contains_point(loc)) {
    ER_WARN("Cache%d@%s invalid for acquisition: does not contain %s",
            id,
            it->ent()->discrete_loc().to_str().c_str(),
            loc.to_str().c_str());
  }

  auto cache = it->ent();
  auto& params = boost::get<params::cache_sel::pickup_policy_params>(
      mc_csel_matrix->find(cselm::kInitialPickupPolicy)->second);

  if (cselm::kInitialPickupPolicyTime == params.policy &&
      timestep < params.timestep) {
    return false;
  } else if (cselm::kInitialPickupPolicyCacheSize == params.policy &&
             cache->n_blocks() < params.cache_size) {
    ER_WARN("Existing cache%d@%s invalid for acquisition: too few blocks (%zu < %u)",
            cache->id(),
            cache->discrete_loc().to_str().c_str(),
            cache->n_blocks(),
            params.cache_size);
    return false;
  }
  return true;
} /* operator()() */

NS_END(fsm, fordyca);
