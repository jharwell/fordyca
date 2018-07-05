/**
 * @file arena_cache.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_ARENA_CACHE_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_ARENA_CACHE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <vector>

#include "fordyca/metrics/caches/utilization_metrics.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class arena_cache
 * @ingroup representation
 *
 * @brief A representation of an ACTUAL cache within the arena. This differs
 * from \ref perceived_cache objects because they handle cache penalties and can
 * collect metrics about their usage.
 */
class arena_cache
    : public base_cache,
      public metrics::caches::utilization_metrics,
      public rcppsw::patterns::visitor::visitable_any<arena_cache> {
 public:
  arena_cache(double dimension,
              double resolution,
              argos::CVector2 center,
              const std::vector<std::shared_ptr<block>>& blocks,
              int id);

  /* metrics */
  uint n_blocks(void) const override { return base_cache::n_blocks(); }
  uint total_block_pickups(void) const override { return m_block_pickups; }
  uint total_block_drops(void) const override { return m_block_drops; }
  uint total_penalties_served(void) const override { return m_penalty_count; }
  void reset_metrics(void) override;
  uint cache_id(void) const override { return id(); }

  void has_block_pickup(void) { m_block_pickups = 1; }
  void has_block_drop(void) { m_block_drops = 1; }
  void penalty_served(uint duration) { m_penalty_count += duration; }

 private:
  // clang-format off
  uint   m_block_pickups{0};
  uint   m_block_drops{0};
  uint   m_penalty_count{0};
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_ARENA_CACHE_HPP_ */
