/**
 * @file base_cache_manager.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include <vector>
#include "fordyca/metrics/caches/lifecycle_metrics.hpp"
#include "rcppsw/common/common.hpp"
#include "fordyca/ds/cache_vector.hpp"
#include "fordyca/ds/block_vector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace representation {
class base_block;
class arena_cache;
} // namespace representation
namespace ds {
class arena_grid;
}

NS_START(support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_cache_manager
 * @ingroup support depth2
 *
 * @brief Manager for creation, depletion, and metric gathering for base
 * caches in the arena, whenever they are enabled.
 */
class base_cache_manager : public metrics::caches::lifecycle_metrics {
 public:
  struct creation_result {
    creation_result(bool b, const ds::cache_vector& c) :
        status(b),
        caches(c) {}

    bool status;
    ds::cache_vector caches;
  };

  struct block_calc_result {
    block_calc_result(bool b, const ds::block_vector& c) :
        status(b),
        blocks(c) {}

    bool status;
    ds::block_vector blocks;
  };

  explicit base_cache_manager(ds::arena_grid* const arena_grid)
      : m_grid(arena_grid) {}
  virtual ~base_cache_manager(void) = default;

  base_cache_manager(const base_cache_manager& other) = delete;
  base_cache_manager& operator=(const base_cache_manager& other) = delete;

  /* cache lifecycle metrics */
  uint caches_created(void) const override { return m_cache_created; }
  uint caches_depleted(void) const override { return m_cache_depleted; }
  void caches_created(uint c) { m_cache_created += c; }
  void caches_depleted(uint c) { m_cache_depleted += c; }
  void reset_metrics(void) override {
    m_cache_created = 0;
    m_cache_depleted = 0;
  }

  void cache_created(void) { ++m_cache_created; }
  void cache_depleted(void) { ++m_cache_depleted; }

 protected:
  const ds::arena_grid* arena_grid(void) const { return m_grid; }
  ds::arena_grid* arena_grid(void) { return m_grid; }

 private:
  // clang-format off
  uint                   m_cache_created{0};
  uint                   m_cache_depleted{0};
  ds::arena_grid * const m_grid;
  // clang-format on
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_CACHE_MANAGER_HPP_ */
