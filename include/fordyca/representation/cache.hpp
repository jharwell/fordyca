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
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "fordyca/metrics/collectible_metrics/cache_metrics.hpp"
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "fordyca/representation/block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace prototype = rcppsw::patterns::prototype;

NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache
 * @ingroup representation
 *
 * @brief A representation of a cache within the arena map. Caches do not have
 * state, and if/when a cache becomes empty, it needs to be deleted by an
 * enclosing class. Caches have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena
 * map).
 */
class cache : public immovable_cell_entity,
              public metrics::collectible_metrics::cache_metrics,
              public rcppsw::patterns::visitor::visitable_any<cache>,
              public prototype::clonable<cache> {
 public:
  /**
   * @param dimension The size of the cache. Does not have to be a multiple of
   * the arena resolution, but doing so makes it easier.
   * @param resolution The arena resolution.
   * @param center (X,Y) coordinates of the center of the cache.
   * @param blocks The initial block list for the cache.
   * @param id The ID to assign to the cache; -1 for a new cache, which
   * will generate a new ID, or any positive # to use the same ID as an existing
   * cache (used when cloning a cache into a robot's perceived arena map).
   */
  cache(double dimension, double resolution,
        argos::CVector2 center,
        std::vector<block*>& blocks,
        int id);

  __pure bool operator==(const cache &other) const {
    return this->discrete_loc() == other.discrete_loc();
  }

  /* metrics */
  size_t n_blocks(void) const override { return m_blocks.size(); }
  size_t n_block_pickups(void) const override { return m_n_block_pickups; }
  size_t n_block_drops(void) const override { return m_n_block_drops; }

  void inc_block_pickups(void) { ++m_n_block_pickups; }
  void inc_block_drops(void) { ++m_n_block_drops; }

  std::unique_ptr<cache> clone(void) const override;

  /**
   * @brief \c TRUE iff the cache contains the specified block.
   */
  __pure bool contains_block(const block* c_block) const {
    return std::find(m_blocks.begin(), m_blocks.end(), c_block) != m_blocks.end();
  }

  /**
   * @brief Get a list of the blocks currently in the cache.
   */
  std::vector<block*>& blocks(void) { return m_blocks; }

  /**
   * @brief Add a new block to the cache's list of blocks.
   *
   * Does not update the block's location.
   */
  void block_add(block* block) { m_blocks.push_back(block);  }

  /**
   * @brief Remove a block from the cache's list of blocks.
   *
   * Does not update the block's location.
   */
  void block_remove(block* block);

  /**
   * @brief Get the oldest block in the cache (the one that has been in the
   * cache the longest).
   */
  block* block_get(void) { return m_blocks.front(); }

 private:
  // clang-format off
  static int          m_next_id;
  double              m_resolution;
  size_t              m_n_block_pickups;
  size_t              m_n_block_drops;
  std::vector<block*> m_blocks;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_CACHE_HPP_ */
