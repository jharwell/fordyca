/**
 * \file base_manager.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/caches/base_manager.hpp"

#include <cmath>

#include "rcppsw/algorithm/transform.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/sim_block3D.hpp"
#include "cosm/spatial/common/dimension_checker.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, caches);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<base_manager::creation_blocks>
base_manager::creation_blocks_alloc(
    const cds::block3D_vectorno& all_blocks,
    const cads::acache_vectorno& existing_caches,
    const cfds::block3D_cluster_vectorro& clusters,
    const block_alloc_filter_type& usable_filter,
    const block_alloc_filter_type& absorbable_filter) {
  creation_blocks allocated;

  std::copy_if(all_blocks.begin(),
               all_blocks.end(),
               std::back_inserter(allocated.usable),
               [&](const auto* b) {
                 return usable_filter(b, existing_caches, clusters);
               });

  auto absorbable_transform = [&](auto* b) { return std::make_pair(b->id(), b); };

  ralg::transform_if(
      all_blocks.begin(),
      all_blocks.end(),
      std::inserter(allocated.absorbable, allocated.absorbable.begin()),
      [&](const auto* b) {
        return absorbable_filter(b, existing_caches, clusters);
      },
      absorbable_transform);

  if (creation_blocks_alloc_check(allocated, existing_caches)) {
    return boost::make_optional(allocated);
  } else {
    ER_FATAL_SENTINEL("Bad creation blocks allocation");
    return boost::none;
  }
} /* creation_blocks_alloc() */

bool base_manager::creation_blocks_alloc_check(
    const creation_blocks& c_allocated,
    const cads::acache_vectorno& c_existing_caches) const {
  if (c_allocated.usable.size() < mc_config.dynamic.min_blocks) {
    /*
     * \todo Cannot use std::accumulate for these, because that doesn't work
     * with C++14/gcc7 when you are accumulating into a different type
     * (e.g. from a set of blocks into an int).
     */
    uint count = 0;
    std::for_each(
        c_allocated.usable.begin(), c_allocated.usable.end(), [&](const auto& b) {
          count +=
              (b->is_out_of_sight() ||
               std::any_of(c_existing_caches.begin(),
                           c_existing_caches.end(),
                           [&](const auto& c) { return !c->contains_block(b); }));
        });

    std::string accum;
    std::for_each(
        c_allocated.usable.begin(), c_allocated.usable.end(), [&](const auto& b) {
          accum += "b" + rcppsw::to_string(b->id()) + "->fb" +
                   rcppsw::to_string(b->md()->robot_id()) + ",";
        });
    ER_DEBUG("Block carry statuses: [%s]", accum.c_str());

    accum = "";
    std::for_each(
        c_allocated.usable.begin(), c_allocated.usable.end(), [&](const auto& b) {
          accum += "b" + rcppsw::to_string(b->id()) + "->" +
                   b->ranchor2D().to_str() + "/" + b->danchor2D().to_str() + ",";
        });
    ER_DEBUG("Block locations: [%s]", accum.c_str());

    ER_CHECK(c_allocated.usable.size() - count < mc_config.dynamic.min_blocks,
             "For new caches, %zu blocks SHOULD be available, but only %zu "
             "are (min=%u)",
             c_allocated.usable.size() - count,
             c_allocated.usable.size(),
             mc_config.dynamic.min_blocks);
  }
  if (c_allocated.usable.size() < mc_config.static_.size) {
    ER_WARN("Free block count < min blocks for new caches (%zu < %u)",
            c_allocated.usable.size(),
            mc_config.dynamic.min_blocks);
  }
  return true;

error:
  return false;
} /* creation_blocks_alloc_check() */

void base_manager::bloctree_update(const cads::acache_vectoro& caches) {
  cads::acache_vectorro created;
  for (const auto& c : caches) {
    created.push_back(c.get());
  } /* for(&b..) */
  m_map->created_caches(created);

  for (const auto& cache : caches) {
    for (auto* block : cache->blocks()) {
      m_map->bloctree_update(block, carena::locking::ekALL_HELD);
    } /* for(*block..) */
  } /* for(&cache..) */

  m_map->created_caches_clear();
} /* bloctree_update() */

rspatial::euclidean_dist base_manager::cache_dim_calc(void) const {
  using checker = cspatial::dimension_checker;

  /*
   * Depending on floating point rounding errors, the value we are holding from
   * parsed XML configuration might be such that when converted to an integer
   * grid index, we get a dimension one less than we intend. This function makes
   * sure that doesn't happen.
   */
  auto even_multiple = checker::even_multiple(arena_map()->grid_resolution(),
                                              config()->dimension);
  ER_ASSERT(rmath::is_multiple_of(even_multiple.v(),
                                  arena_map()->grid_resolution().v()),
            "Cache dimension not a multiple of grid resolution: %.12f %% %.12f != 0",
            even_multiple.v(),
            arena_map()->grid_resolution().v());

  auto odd_dsize = checker::odd_dsize(arena_map()->grid_resolution(),
                                      even_multiple);

  /* Caches must be odd in X,Y so the center is well defined */
  auto int_dim = static_cast<int>(odd_dsize.v() / arena_map()->grid_resolution().v());
  ER_ASSERT(RCPPSW_IS_ODD(int_dim),
            "Cache dimension has no defined center: %.12f/%.12f=%d is even",
            odd_dsize.v(),
            arena_map()->grid_resolution().v(),
            int_dim);
  return odd_dsize;
} /* cache_dim_calc() */

NS_END(caches, support, argos, fordyca);
