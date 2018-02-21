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
#include <utility>
#include <vector>

#include "fordyca/metrics/cache_metrics.hpp"
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
class arena_cache : public base_cache,
                    public metrics::collectible_metrics::cache_metrics,
                    public rcppsw::patterns::visitor::visitable_any<arena_cache> {
 public:
  arena_cache(double dimension,
              double resolution,
              argos::CVector2 center,
              std::vector<block*>& blocks,
              int id);

  /* metrics */
  uint n_blocks(void) const override { return base_cache::n_blocks(); }
  uint n_block_pickups(void) const override { return m_n_block_pickups; }
  uint n_block_drops(void) const override { return m_n_block_drops; }
  uint penalties_served(void) const override { return m_penalty_count; }
  void reset_metrics(void) override;

  void inc_block_pickups(void) { ++m_n_block_pickups; }
  void inc_block_drops(void) { ++m_n_block_drops; }

 private:
  // clang-format off
  uint   m_n_block_pickups{0};
  uint   m_n_block_drops{0};
  uint   m_penalty_count{0};
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_ARENA_CACHE_HPP_ */
