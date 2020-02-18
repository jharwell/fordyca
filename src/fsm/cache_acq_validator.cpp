/**
 * \file cache_acq_validator.cpp
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
#include "fordyca/fsm/cache_acq_validator.hpp"

#include "cosm/foraging/repr/base_cache.hpp"

#include "fordyca/config/cache_sel/cache_pickup_policy_config.hpp"
#include "fordyca/controller/cache_sel_matrix.hpp"
#include "fordyca/ds/dp_cache_map.hpp"
#include "cosm/foraging/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);
using cselm = controller::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_acq_validator::cache_acq_validator(
    const ds::dp_cache_map* map,
    const controller::cache_sel_matrix* csel_matrix,
    bool for_pickup)
    : ER_CLIENT_INIT("fordyca.fsm.cache_acq_validator"),
      mc_for_pickup(for_pickup),
      mc_csel_matrix(csel_matrix),
      mc_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool cache_acq_validator::operator()(const rmath::vector2d& loc,
                                     const rtypes::type_uuid& id,
                                     const rtypes::timestep& t) const {
  /*
   * We can't just lookup the cache by the location key we are passed directly,
   * as it is for a point somewhere *inside* the cache, and thus probably not at
   * the cache's host cell location. Instead we look up the cache by ID, and
   * verify that the cache exists contains the point we are acquiring.
   */
  auto range = mc_map->const_values_range();
  auto it = std::find_if(range.begin(), range.end(), [&](const auto& c) {
    return c.ent()->id() == id;
  });

  if (range.end() == it) {
    ER_WARN("Cache%d near %s invalid for acquisition: cache unknown",
            id.v(),
            loc.to_str().c_str());
    return false;
  } else if (!it->ent()->contains_point(loc)) {
    ER_WARN("Cache%d@%s invalid for acquisition: does not contain %s",
            id.v(),
            it->ent()->dloc().to_str().c_str(),
            loc.to_str().c_str());
    return false;
  }

  /*
   * As long as (1) the cache exists, (2) the point we are trying to acquire is
   * inside it, (3) we are dropping something in the cache, we can go ahead and
   * acquire it without further checking.
   */
  if (!mc_for_pickup) {
    return true;
  }

  /* verify pickup policy */
  return pickup_policy_validate(it->ent(), t);
} /* operator()() */

bool cache_acq_validator::pickup_policy_validate(
    const cfrepr::base_cache* cache,
    const rtypes::timestep& t) const {
  auto& config = boost::get<config::cache_sel::cache_pickup_policy_config>(
      mc_csel_matrix->find(cselm::kPickupPolicy)->second);

  if (cselm::kPickupPolicyTime == config.policy && t < config.timestep) {
    ER_WARN("Cache%d invalid for acquisition: policy=%s, %u < %u",
            cache->id().v(),
            config.policy.c_str(),
            t.v(),
            config.timestep.v());
    return false;
  } else if (cselm::kPickupPolicyCacheSize == config.policy &&
             cache->n_blocks() < config.cache_size) {
    ER_WARN("Cache%d invalid for acquisition: policy=%s, %zu < %u",
            cache->id().v(),
            config.policy.c_str(),
            cache->n_blocks(),
            config.cache_size);
    return false;
  }
  return true;
} /* pickup_policy_validate() */

NS_END(fsm, fordyca);
