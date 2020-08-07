/**
 * \file base_cache_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <mutex>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/ds/block3D_vector.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/metrics/caches/lifecycle_metrics.hpp"

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
 * \class base_cache_manager
 * \ingroup support depth2
 *
 * \brief Manager for creation, depletion, and metric gathering for base
 * caches in the arena, whenever they are enabled.
 */
class base_cache_manager : public metrics::caches::lifecycle_metrics,
                           public rer::client<base_cache_manager> {
 public:
  explicit base_cache_manager(carena::caching_arena_map* const map)
      : ER_CLIENT_INIT("fordyca.support.cache_manager"), m_map(map) {}
  ~base_cache_manager(void) override = default;

  base_cache_manager(const base_cache_manager&) = delete;
  base_cache_manager& operator=(const base_cache_manager&) = delete;

  /* cache lifecycle metrics */
  size_t caches_created(void) const override final { return m_caches_created; }
  size_t caches_discarded(void) const override final { return m_caches_discarded; }
  size_t caches_depleted(void) const override final {
    return m_depletion_ages.size();
  }
  std::vector<rtypes::timestep> cache_depletion_ages(void) const override {
    return m_depletion_ages;
  }

  void cache_depleted(const rtypes::timestep& age) {
    m_depletion_ages.push_back(age);
  }
  void reset_metrics(void) override final {
    m_caches_created = 0;
    m_caches_discarded = 0;
    m_depletion_ages.clear();
  }
  std::mutex& mtx(void) { return m_mutex; }

 protected:
  /**
   * \brief Check the dimension that a derived class wants to use to create
   * caches with, and modify it if necessary.
   *
   * Invariant: Caches are an odd # of cells in X,Y so that they have a uniquely
   * defined discrete center.
   *
   * If the cache dimension is not an odd multiple of the arena resolution, then
   * a modified dimension is returned (smaller than the argument, never larger).
   */
  rtypes::spatial_dist dimension_check(rtypes::spatial_dist dim) const;

  void caches_created(size_t c) { m_caches_created += c; }
  void caches_discarded(size_t c) { m_caches_discarded += c; }
  const carena::caching_arena_map* arena_map(void) const { return m_map; }
  carena::caching_arena_map* arena_map(void) { return m_map; }

  /**
   * \brief Update the arena map bloctree as a result of successful cache
   * creation.
   *
   * \param caches Vector of newly created caches.
   */
  void bloctree_update(const cads::acache_vectoro& caches);

 private:
  /* clang-format off */
  size_t                            m_caches_created{0};
  size_t                            m_caches_discarded{0};
  std::vector<rtypes::timestep>     m_depletion_ages{};
  carena::caching_arena_map * const m_map;
  std::mutex                        m_mutex{};
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_MANAGER_HPP_ */
