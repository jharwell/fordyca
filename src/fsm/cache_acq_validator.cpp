/**
 * \file cache_acq_validator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fsm/cache_acq_validator.hpp"

#include "cosm/arena/repr/base_cache.hpp"

#include "fordyca/controller/cognitive/cache_sel_matrix.hpp"
#include "fordyca/controller/config/cache_sel/cache_pickup_policy_config.hpp"
#include "fordyca/subsystem/perception/ds/dp_cache_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, fsm);
using cselm = controller::cognitive::cache_sel_matrix;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
cache_acq_validator::cache_acq_validator(
    const fspds::dp_cache_map* dpo_map,
    const controller::cognitive::cache_sel_matrix* csel_matrix,
    bool for_pickup)
    : ER_CLIENT_INIT("fordyca.fsm.cache_acq_validator"),
      mc_for_pickup(for_pickup),
      mc_csel_matrix(csel_matrix),
      mc_caches(fspds::dp_cache_map::raw_values_extract<cads::bcache_vectorno>(
          *dpo_map)) {}

cache_acq_validator::cache_acq_validator(
    const cads::bcache_vectorno& caches,
    const controller::cognitive::cache_sel_matrix* csel_matrix,
    bool for_pickup)
    : ER_CLIENT_INIT("fordyca.fsm.cache_acq_validator"),
      mc_for_pickup(for_pickup),
      mc_csel_matrix(csel_matrix),
      mc_caches(caches) {}

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
  auto it = std::find_if(mc_caches.begin(), mc_caches.end(), [&](const auto& c) {
    return c->id() == id;
  });

  if (mc_caches.end() == it) {
    ER_WARN("Cache%d near %s invalid for acquisition: cache unknown",
            id.v(),
            loc.to_str().c_str());
    return false;
  } else if (!(*it)->contains_point(loc)) {
    ER_WARN("Cache%d@%s invalid for acquisition: does not contain %s",
            id.v(),
            rcppsw::to_string((*it)->dcenter2D()).c_str(),
            rcppsw::to_string(loc).c_str());
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
  return pickup_policy_validate(*it, t);
} /* operator()() */

bool cache_acq_validator::pickup_policy_validate(const carepr::base_cache* cache,
                                                 const rtypes::timestep& t) const {
  const auto& config = std::get<fcconfig::cache_sel::cache_pickup_policy_config>(
      mc_csel_matrix->find(cselm::kPickupPolicy)->second);

  if (cselm::kPickupPolicyTime == config.policy && t < config.timestep) {
    ER_DEBUG("Cache%d invalid for acquisition: policy=%s, %zu < %zu",
             cache->id().v(),
             config.policy.c_str(),
             t.v(),
             config.timestep.v());
    return false;
  } else if (cselm::kPickupPolicyCacheSize == config.policy &&
             cache->n_blocks() < config.cache_size) {
    ER_DEBUG("Cache%d invalid for acquisition: policy=%s, %zu < %zu",
             cache->id().v(),
             config.policy.c_str(),
             cache->n_blocks(),
             config.cache_size);
    return false;
  } else if (cselm::kPickupPolicyCacheDuration == config.policy &&
             t - cache->creation_ts() < config.timestep) {
    ER_DEBUG("Cache%d invalid for acquisition: policy=%s, %zu < %zu",
             cache->id().v(),
             config.policy.c_str(),
             (t - cache->creation_ts()).v(),
             config.timestep.v());
    return false;
  }
  return true;
} /* pickup_policy_validate() */

NS_END(fsm, fordyca);
