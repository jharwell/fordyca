/**
 * \file base_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <mutex>
#include <vector>
#include <boost/optional.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/ds/block3D_vector.hpp"
#include "cosm/ds/block3D_ht.hpp"
#include "cosm/foraging/ds/block_cluster_vector.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/metrics/caches/lifecycle_metrics.hpp"
#include "fordyca/argos/support/caches/config/caches_config.hpp"

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
 * \class base_manager
 * \ingroup argos support caches
 *
 * \brief Manager for creation, depletion, and metric gathering for base
 * caches in the arena, whenever they are enabled.
 */
class base_manager : public fmetrics::caches::lifecycle_metrics,
                     public rer::client<base_manager> {
 public:
  base_manager(const fascaches::config::caches_config* config,
               carena::caching_arena_map* const map)
      : ER_CLIENT_INIT("fordyca.argos.support.cache_manager"),
        mc_config(*config),
        m_map(map) {}
  ~base_manager(void) override = default;

  base_manager(const base_manager&) = delete;
  base_manager& operator=(const base_manager&) = delete;

  /* cache lifecycle metrics */
  size_t caches_created(void) const override final { return m_caches_created; }
  size_t caches_discarded(void) const override final {
    return m_caches_discarded;
  }
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
  struct creation_blocks {
    cds::block3D_vectorno usable{};
    cds::block3D_htno absorbable{};
  };
  using block_alloc_filter_type = std::function<bool(
      const crepr::sim_block3D* block,
      const cads::acache_vectorno& existing_caches,
      const cfds::block3D_cluster_vectorro& clusters)>;

  boost::optional<creation_blocks> creation_blocks_alloc(
      const cds::block3D_vectorno& all_blocks,
      const cads::acache_vectorno& existing_caches,
      const cfds::block3D_cluster_vectorro& clusters,
      const block_alloc_filter_type& usable_filter,
      const block_alloc_filter_type& absorbable_filter);

  bool creation_blocks_alloc_check(const creation_blocks& c_allocated,
                                   const cads::acache_vectorno& c_existing_caches) const;

  /**
   * \brief Calculate the run-time size of the cache from configuration.
   *
   *
   * The calculated value must be:
   *
   * - An odd multiple of the  \ref base_arena_map grid resolution so that
   *   caches have an unambiguous center.
   */
  rspatial::euclidean_dist cache_dim_calc(void) const;

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
  rspatial::euclidean_dist dimension_check(rspatial::euclidean_dist dim) const;

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

  const fascaches::config::caches_config* config(void) const { return &mc_config; }

 private:
  /* clang-format off */
  const fascaches::config::caches_config mc_config;

  size_t                                 m_caches_created{0};
  size_t                                 m_caches_discarded{0};
  std::vector<rtypes::timestep>          m_depletion_ages{};

  carena::caching_arena_map * const      m_map;
  std::mutex                             m_mutex{};
  /* clang-format on */
};

NS_END(caches, support, argos, fordyca);

