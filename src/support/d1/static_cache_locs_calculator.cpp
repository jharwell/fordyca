/**
 * \file static_cache_locs_calculator.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/d1/static_cache_locs_calculator.hpp"

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/block_dist/dispatcher.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/nest.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, d1);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::vector<rmath::vector2d> static_cache_locs_calculator::operator()(
    const carena::caching_arena_map* arena_map,
    const cfconfig::block_dist_config* distp) {
  using dispatcher_type = cfbd::dispatcher;

  std::vector<rmath::vector2d> cache_rlocs;
  ER_ASSERT(1 == arena_map->nests().size(),
            "Multiple nests incompatible with static cache management");
  const auto* nest = arena_map->nest(rtypes::type_uuid(0));

  /*
   * For the single source, dual source, quad source block distributions, each
   * of the static caches is halfway between the center of the nest and a block
   * cluster. Arena map initialization has already happened at this point, as
   * the created block clusters are needed.
   */
  if (dispatcher_type::kDistSingleSrc == distp->dist_type ||
      dispatcher_type::kDistDualSrc == distp->dist_type) {
    auto clusters = arena_map->block_distributor()->block_clustersro();

    for (auto& c : clusters) {
      cache_rlocs.push_back(
          { (c->xrspan().center() + nest->rcenter2D().x()) / 2.0,
            (c->yrspan().center() + nest->rcenter2D().y()) / 2.0 });
    } /* for(i..) */
  } else if (dispatcher_type::kDistQuadSrc == distp->dist_type) {
    /*
     * Quad source is a tricky distribution to use with static caches, so we
     * have to tweak the static cache locations in tandem with the block cluster
     * locations to ensure that no segfaults results from cache/cache or
     * cache/cluster overlap. See FORDYCA#581.
     *
     * Basically we want the cache centers to be halfway between the nest center
     * and each of the block cluster centers (we assume a square arena).
     */
    auto clusters = arena_map->block_distributor()->block_clustersro();
    for (auto& c : clusters) {
      bool on_center_y = std::fabs(c->xrspan().center() - nest->rcenter2D().x()) <
                         0.5;
      bool on_center_x = std::fabs(c->yrspan().center() - nest->rcenter2D().y()) <
                         0.5;
      ER_ASSERT(on_center_y || on_center_x,
                "Cluster@%f,%f not centered in arena X or Y",
                c->xrspan().center(),
                c->yrspan().center());
      if (on_center_x &&
          c->xrspan().center() < nest->rcenter2D().x()) { /* west */
        cache_rlocs.push_back(
            { arena_map->xrsize() * 0.30, c->yrspan().center() });
      } else if (on_center_x &&
                 c->xrspan().center() > nest->rcenter2D().x()) { /* east */
        cache_rlocs.push_back(
            { arena_map->xrsize() * 0.675, c->yrspan().center() });
      } else if (on_center_y &&
                 c->yrspan().center() < nest->rcenter2D().y()) { /* south */
        cache_rlocs.push_back(
            { c->xrspan().center(), arena_map->yrsize() * 0.30 });
      } else if (on_center_y &&
                 c->yrspan().center() > nest->rcenter2D().y()) { /* north */
        cache_rlocs.push_back(
            { c->xrspan().center(), arena_map->yrsize() * 0.675 });
      }
    } /* for(i..) */
  } else if (dispatcher_type::kDistPowerlaw == distp->dist_type ||
             dispatcher_type::kDistRandom == distp->dist_type) {
    /* west */
    cache_rlocs.push_back(
        { arena_map->xrsize() * 0.25, arena_map->yrsize() * 0.5 });

    /* east */
    cache_rlocs.push_back(
        { arena_map->xrsize() * 0.75, arena_map->yrsize() * 0.5 });

    /* south */
    cache_rlocs.push_back(
        { arena_map->xrsize() * 0.5, arena_map->yrsize() * 0.25 });

    /* north */
    cache_rlocs.push_back(
        { arena_map->xrsize() * 0.5, arena_map->yrsize() * 0.75 });
  } else {
    ER_FATAL_SENTINEL("Block distribution '%s' unsupported for static cache "
                      "management",
                      distp->dist_type.c_str());
  }
  /*
   * For all cache locs, transform real -> discrete to ensure the real and
   * discrete cache centers used during simulation are convertible without loss
   * of precision/weird corner cases. Add 1/2 cell width to them so that the
   * real center is in the center of the host cell, and not the LL corner.
   */
  std::vector<rmath::vector2d> cache_rcenters;
  rmath::vector2d offset(arena_map->grid_resolution().v() / 2.0,
                         arena_map->grid_resolution().v() / 2.0);
  std::transform(
      cache_rlocs.begin(),
      cache_rlocs.end(),
      std::back_inserter(cache_rcenters),
      [&](const auto& rloc) {
        auto tmp = rmath::dvec2zvec(rloc, arena_map->grid_resolution().v());
        auto ll = rmath::zvec2dvec(tmp, arena_map->grid_resolution().v());
        return ll + offset;
      });
  return cache_rcenters;
} /* operator()() */

NS_END(d1, support, fordyca);
