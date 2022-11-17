/**
 * \file base_creator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/arena/ds/nest_vector.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/ds/block3D_ht.hpp"

#include "fordyca/argos/support/caches/create_ro_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, argos, support, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_creator
 * \ingroup argos support caches
 *
 * \brief Base class for creating caches in the arena.
 */
class base_creator : public rer::client<base_creator> {
 public:
  struct creation_result {
    cads::acache_vectoro created{};
    size_t n_discarded{};
  };

  /**
   * \brief Initialize a new cache creator.
   *
   * \param map Reference to arena map.
   * \param cache_dim Dimension of the cache (caches are square so can use a
   *                  scalar).
   */
  base_creator(carena::caching_arena_map* map,
                     const rspatial::euclidean_dist& cache_dim);

  base_creator(const base_creator&) = delete;
  base_creator& operator=(const base_creator&) = delete;

  /**
   * \brief Configure the the cache extent cells for all newly created caches.
   *
   * \param caches Vector of newly created caches.
   */
  void cache_extents_configure(const cads::acache_vectoro& caches);

 protected:
  const carena::caching_arena_map* map(void) const { return m_map; }
  carena::caching_arena_map* map(void) { return m_map; }

  /**
   * \brief Create a single cache in the arena from the specified set of blocks
   * at the specified location.
   *
   * \param center Location of the new cache.
   * \param blocks Vector of blocks to use to create the cache. Passed by value
   *               because they are (possibly) modified by this function in a
   *               way that callers probably do not want.
   * \param t The current timestep.
   */
  std::shared_ptr<carepr::arena_cache>
  create_single_cache(const rmath::vector2d& center,
                      cds::block3D_vectorno&& blocks,
                      const rtypes::timestep& t,
                      bool pre_dist);

  rspatial::euclidean_dist cache_dim(void) const { return mc_cache_dim; }

 private:
  /* clang-format off */
  const rspatial::euclidean_dist mc_cache_dim;

  carena::caching_arena_map* m_map;
  /* clang-format on */
};
NS_END(caches, support, argos, fordyca);

