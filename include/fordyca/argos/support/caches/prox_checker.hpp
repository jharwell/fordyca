/**
 * \file prox_checker.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/controller/cognitive/d2/events/cache_proximity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, caches);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class prox_checker
 * \ingroup argos support caches
 *
 * \brief Check if a controller is too close to a cache for a block drop of some
 * kind.
 */
class prox_checker : public rer::client<prox_checker> {
 public:
  struct proximity_status {
    rtypes::type_uuid id{ rtypes::constants::kNoUUID };
    rmath::vector2d loc{};
    rmath::vector2d distance{};
  };

  prox_checker(const carena::caching_arena_map* const map,
                     const rspatial::euclidean_dist& prox_dist)
      : ER_CLIENT_INIT("fordyca.argos.support.d2.prox_checker"),
        mc_prox_dist(prox_dist),
        mc_map(map) {}

  /* Not copy constructable/assignable by default */
  prox_checker(const prox_checker&) = delete;
  const prox_checker& operator=(const prox_checker&) = delete;

  prox_checker(prox_checker&&) = default;
  prox_checker& operator=(prox_checker&&) = delete;

  /**
   * \brief Determine if creating a new cache centered at the robot's current
   * position will overlap with any other caches in the arena/be too close to
   * them.
   *
   * \note This is an approximate check, because the weighted centroid of
   * constituent blocks is used rather than the robot's current location when
   * creating a new cache, but this should serve as a good check against invalid
   * cache creation.
   *
   * \return (cache id of cache that is too close (-1 if none), distance to said
   *         cache).
   */
  proximity_status check(const controller::foraging_controller& c,
                         bool need_lock = true) const {
    proximity_status result;

    /*
     * OK to lock around this calculation, because relatively few robots will
     * have the correct internal state for a cache operation each timestep, and
     * we don't need exclusive access to the arena map at this point--only a
     * guarantee that the cache array will not be modified while we are checking
     * it.
     */
    mc_map->maybe_lock_rd(mc_map->cache_mtx(), need_lock);
    for (const auto* cache : mc_map->caches()) {
      if (mc_prox_dist >= (cache->rcenter2D() - c.rpos2D()).length()) {
        result = { cache->id(),
                   cache->rcenter2D(),
                   cache->rcenter2D() - c.rpos2D() };
        break;
      }
    } /* for(&b..) */
    mc_map->maybe_unlock_rd(mc_map->cache_mtx(), need_lock);
    return result;
  }

  template <typename TController>
  bool check_and_notify(TController& controller,
                        RCPPSW_UNUSED const std::string& drop_dest) const {
    mc_map->lock_rd(mc_map->cache_mtx());

    /*
     * Check again if there is a cache too close, and we were not holding the
     * cache mutex between the check during penalty initialization and now
     * (making them atomic), so even if we DIDN'T think there was a cache too
     * close before, there might be one too close now.
     */
    auto prox_status = check(controller, false);

    /*
     * We are holding the cache mutex, so if our check says that there is no
     * cache too close, we can trust it.
     */
    if (rtypes::constants::kNoUUID == prox_status.id) {
      mc_map->unlock_rd(mc_map->cache_mtx());
      return false;
    } else {
      ER_WARN("Robot%d@%s cannot drop block in %s: Cache%d@%s too close (%f <= "
              "%f)",
              controller.entity_id().v(),
              controller.rpos2D().to_str().c_str(),
              drop_dest.c_str(),
              prox_status.id.v(),
              prox_status.loc.to_str().c_str(),
              prox_status.distance.length(),
              mc_prox_dist.v());
      notify(controller, prox_status.id, false);

      mc_map->unlock_rd(mc_map->cache_mtx());
      return true;
    }
  }

  template <typename TController>
  bool notify(TController& controller,
              const rtypes::type_uuid& cache_id,
              bool need_lock = true) const {
    mc_map->maybe_lock_rd(mc_map->cache_mtx(), need_lock);

    /*
       * Because caches can be dynamically created/destroyed, we cannot rely on
       * the index position of cache i to be the same as its ID, so we need to
       * search for the correct cache.
       */
    auto it = std::find_if(mc_map->caches().begin(),
                           mc_map->caches().end(),
                           [&](const auto& c) { return c->id() == cache_id; });

    fccd2::events::cache_proximity_visitor prox_op(*it);
    prox_op.visit(controller);
    mc_map->maybe_unlock_rd(mc_map->cache_mtx(), need_lock);
    return true;
  }

 private:
  /* clang-format off */
  const rspatial::euclidean_dist       mc_prox_dist;
  const carena::caching_arena_map* mc_map;
  /* clang-format on */
};

NS_END(caches, support, argos, fordyca);

