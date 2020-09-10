/**
 * \file cache_prox_checker.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CACHE_PROX_CHECKER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CACHE_PROX_CHECKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/events/cache_proximity.hpp"
#include "fordyca//controller/foraging_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class cache_prox_checker
 * \ingroup support
 *
 * \brief Check if a controller is too close to a cache for a block drop of some
 * kind.
 *
 * \note This can't be part of the other event utils, because you can't forward
 * declare things because it has to be a template function.
 */
class cache_prox_checker: public rer::client<cache_prox_checker> {
 public:
  struct proximity_status {
    rtypes::type_uuid id{rtypes::constants::kNoUUID};
    rmath::vector2d loc{};
    rmath::vector2d distance{};
  };

  cache_prox_checker(const carena::caching_arena_map* const map,
                     const rtypes::spatial_dist& prox_dist)
      : ER_CLIENT_INIT("fordyca.support.d2.cache_prox_checker"),
        mc_prox_dist(prox_dist),
        mc_map(map) {}

  /* Not copy constructable/assignable by default */
  cache_prox_checker(const cache_prox_checker&) = delete;
  const cache_prox_checker& operator=(const cache_prox_checker&) = delete;

  cache_prox_checker(cache_prox_checker&&) = default;
  cache_prox_checker& operator=(cache_prox_checker&&) = delete;


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
        result = {cache->id(),
                  cache->rcenter2D(),
                  cache->rcenter2D() - c.rpos2D()};
        break;
      }
    } /* for(&b..) */
    mc_map->maybe_unlock_rd(mc_map->cache_mtx(), need_lock);
    return result;
  }

  template<typename TController>
  bool check_and_notify(TController& controller,
                        RCSW_UNUSED const std::string& drop_dest) const {
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
      ER_WARN("Robot%d@%s cannot drop block in %s: Cache%d@%s too close (%f <= %f)",
              controller.entity_id().v(),
              controller.rpos2D().to_str().c_str(),
              drop_dest.c_str(),
              prox_status.id.v(),
              prox_status.loc.to_str().c_str(),
              prox_status.distance.length(),
              mc_prox_dist.v());

      /*
       * Because caches can be dynamically created/destroyed, we cannot rely on
       * the index position of cache i to be the same as its ID, so we need to
       * search for the correct cache.
       */
      auto it = std::find_if(mc_map->caches().begin(),
                             mc_map->caches().end(),
                             [&](const auto& c) {
                               return c->id() == prox_status.id;
                             });


      events::cache_proximity_visitor prox_op(*it);
      prox_op.visit(controller);
      mc_map->unlock_rd(mc_map->cache_mtx());
      return true;
    }
  }

 private:
  /* clang-format off */
  const rtypes::spatial_dist       mc_prox_dist;
  const carena::caching_arena_map* mc_map;
  /* clang-format on */
};


NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_CACHE_PROX_CHECKER_HPP_ */
