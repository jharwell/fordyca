/**
 * @file cache.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_CACHE_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_CACHE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <utility>
#include <algorithm>

#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/metrics/collectible_metrics/cache_metrics.hpp"
#include "fordyca/representation/cell_entity.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache
 *
 * @brief A representation of a cache within the arena map. Caches do not have
 * state, and if/when a cache becomes empty, it needs to be deleted by an
 * enclosing class. Caches have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena
 * map).
 */
class cache : public cell_entity,
              public metrics::collectible_metrics::cache_metrics,
              public rcppsw::patterns::visitor::visitable_any<cache>,
              public rcppsw::patterns::protype::clonable {
 public:
  cache(double dimension, argos::CVector2 center, std::vector<block*> blocks);

  /* metrics */
  size_t n_blocks(void) const override { return m_blocks.size(); }
  size_t n_block_pickups(void) const override { return m_n_block_pickups; }
  size_t n_block_drops(void) const override { return m_n_block_drops; }

  void inc_block_pickups(void) { ++m_n_block_pickups; }
  void inc_block_drops(void) { ++m_n_block_drops; }

  std::unique_ptr<
  /**
   * @brief \c TRUE iff the cache contains the specified block.
   *
   * @param block
   *
   * @return
   */
  __pure bool contains_block(const block* const block) const {
    return std::find(m_blocks.begin(), m_blocks.end(), block) != m_blocks.end();
  }
  /**
   * @brief Get a list of the blocks currently in the cache.
   */
  std::vector<block*>& blocks(void) { return m_blocks; }

  void block_add(block* block) { m_blocks.push_back(block);  }
  void block_remove(block* block);
  block* block_get(void) { return m_blocks.front(); }

  __pure bool operator==(const cache &other) const {
    return cell_entity::real_loc() == other.cell_entity::real_loc() &&
        m_blocks == other.m_blocks;
  }

 private:
  static int        m_next_id;
  size_t            m_n_block_pickups;
  size_t            m_n_block_drops;
  std::vector<block*> m_blocks;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_CACHE_HPP_ */
