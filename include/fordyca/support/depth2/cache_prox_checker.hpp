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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_PROX_CHECKER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_PROX_CHECKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/arena/caching_arena_map.hpp"

#include "fordyca/events/cache_proximity.hpp"
#include "fordyca/support/utils/event_utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class cache_prox_checker
 * \ingroup support depth2
 *
 * \brief Check if a controller is too close to a cache for a block drop of some
 * kind.
 *
 * \note This can't be part of the other event utils, because you can't forward
 * declare things because it has to be a template function.
 */
class cache_prox_checker: public rer::client<cache_prox_checker> {
 public:
  cache_prox_checker(carena::caching_arena_map* const map_in,
                     const rtypes::spatial_dist& prox_dist)
      : ER_CLIENT_INIT("fordyca.support.depth2.cache_prox_checker"),
        mc_prox_dist(prox_dist),
        m_map(map_in) {}

  /* Not copy constructable/assignable by default */
  cache_prox_checker(const cache_prox_checker&) = delete;
  const cache_prox_checker& operator=(const cache_prox_checker&) = delete;

  cache_prox_checker(cache_prox_checker&&) = default;
  cache_prox_checker& operator=(cache_prox_checker&&) = delete;

  template<typename TController>
  bool operator()(TController& controller,
                  RCSW_UNUSED const std::string& drop_dest) const {
    m_map->cache_mtx()->lock();
    /*
     * Check again if there is a cache too close, and we were not holding the
     * cache mutex between the check during penalty initialization and now
     * (making them atomic), so even if we DIDN'T think there was a cache too
     * close before, there might be one too close now.
     */
    auto prox_status = utils::new_cache_cache_proximity(controller,
                                                        *m_map,
                                                        mc_prox_dist);
    /*
     * We are holding the cache mutex, so if our check says that there is no
     * cache too close, we can trust it.
     */
    if (rtypes::constants::kNoUUID == prox_status.entity_id) {
      return false;
    }

    ER_WARN("Robot%d@%s cannot drop block in %s: Cache%d@%s too close (%f <= %f)",
            controller.entity_id().v(),
            controller.rpos2D().to_str().c_str(),
            drop_dest.c_str(),
            prox_status.entity_id.v(),
            prox_status.entity_loc.to_str().c_str(),
            prox_status.distance.length(),
            mc_prox_dist.v());

    /*
     * Because caches can be dynamically created/destroyed, we cannot rely on
     * the index position of cache i to be the same as its ID, so we need to
     * search for the correct cache.
     */
    auto it = std::find_if(m_map->caches().begin(),
                           m_map->caches().end(),
                           [&](const auto& c) {
                             return c->id() == prox_status.entity_id;
                           });

    events::cache_proximity_visitor prox_op(*it);
    prox_op.visit(controller);
    m_map->cache_mtx()->unlock();
    return true;
  }

 private:
  /* clang-format off */
  const rtypes::spatial_dist mc_prox_dist;

  carena::caching_arena_map* m_map;
  /* clang-format on */
};


NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_CACHE_PROX_CHECKER_HPP_ */
