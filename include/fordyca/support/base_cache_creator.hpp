/**
 * \file base_cache_creator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/arena/ds/nest_vector.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/ds/block3D_ht.hpp"

#include "fordyca/support/cache_create_ro_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class caching_arena_map;
} /* namespace cosm::arena */

NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_cache_creator
 * \ingroup support
 *
 * \brief Base class for creating caches in the arena.
 */
class base_cache_creator : public rer::client<base_cache_creator> {
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
  base_cache_creator(carena::caching_arena_map* map,
                     const rtypes::spatial_dist& cache_dim);

  base_cache_creator(const base_cache_creator&) = delete;
  base_cache_creator& operator=(const base_cache_creator&) = delete;

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
  std::unique_ptr<carepr::arena_cache>
  create_single_cache(const rmath::vector2d& center,
                      cds::block3D_vectorno&& blocks,
                      const rtypes::timestep& t,
                      bool pre_dist);

  rtypes::spatial_dist cache_dim(void) const { return mc_cache_dim; }

 private:
  /* clang-format off */
  const rtypes::spatial_dist mc_cache_dim;

  carena::caching_arena_map* m_map;
  /* clang-format on */
};
NS_END(support, fordyca);

#endif // INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_CREATOR_HPP_
