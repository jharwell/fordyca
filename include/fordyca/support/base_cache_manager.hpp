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

#include "cosm/ds/block2D_vector.hpp"
#include "cosm/arena/ds/cache_vector.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/metrics/caches/lifecycle_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds {
class arena_grid;
} /* namespace cosm::ds */

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
  explicit base_cache_manager(cds::arena_grid* const arena_grid)
      : ER_CLIENT_INIT("fordyca.support.cache_manager"), m_grid(arena_grid) {}
  ~base_cache_manager(void) override = default;

  base_cache_manager(const base_cache_manager&) = delete;
  base_cache_manager& operator=(const base_cache_manager&) = delete;

  /* cache lifecycle metrics */
  uint caches_created(void) const override final { return m_caches_created; }
  uint caches_depleted(void) const override final {
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
    m_depletion_ages.clear();
  }
  std::mutex& mtx(void) { return m_mutex; }

 protected:
  void caches_created(uint c) { m_caches_created += c; }
  const cds::arena_grid* arena_grid(void) const { return m_grid; }
  cds::arena_grid* arena_grid(void) { return m_grid; }

 private:
  /* clang-format off */
  uint                          m_caches_created{0};
  std::vector<rtypes::timestep> m_depletion_ages{};
  cds::arena_grid * const       m_grid;
  std::mutex                    m_mutex{};
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_MANAGER_HPP_ */
