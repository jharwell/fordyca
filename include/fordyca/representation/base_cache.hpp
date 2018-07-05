/**
 * @file base_cache.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_BASE_CACHE_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_BASE_CACHE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

#include "fordyca/representation/block.hpp"
#include "fordyca/representation/immovable_cell_entity.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"
#include "fordyca/representation/multicell_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace prototype = rcppsw::patterns::prototype;

NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_cache
 * @ingroup representation
 *
 * @brief Base class for representating a cache within the arena. Caches do not
 * have state, and if/when a cache becomes empty, it needs to be deleted by an
 * enclosing class. Caches have both real (where they actually live in the
 * world) and discretized locations (where they are mapped to within the arena
 * map).
 */
class base_cache : public multicell_entity,
                   public immovable_cell_entity,
                   public prototype::clonable<base_cache> {
 public:
  /**
   * @brief The minimum # of blocks required for a cache to exist (less than
   * this and you just have a bunch of blocks)
   */
  static constexpr uint kMinBlocks = 2;

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
  base_cache(double dimension,
             double resolution,
             argos::CVector2 center,
             const std::vector<std::shared_ptr<block>>& blocks,
             int id);

  __rcsw_pure bool operator==(const base_cache& other) const {
    return this->discrete_loc() == other.discrete_loc();
  }

  /**
   * @brief \c TRUE iff the cache contains the specified block.
   */
  __rcsw_pure bool contains_block(const std::shared_ptr<block> c_block) const {
    return std::find(m_blocks.begin(), m_blocks.end(), c_block) !=
           m_blocks.end();
  }
  uint n_blocks(void) const { return blocks().size(); }

  /**
   * @brief Get a list of the blocks currently in the cache.
   */
  std::vector<std::shared_ptr<block>>& blocks(void) { return m_blocks; }
  const std::vector<std::shared_ptr<block>>& blocks(void) const {
    return m_blocks;
  }

  /**
   * @brief Add a new block to the cache's list of blocks.
   *
   * Does not update the block's location.
   */
  void block_add(const std::shared_ptr<block>& block) {
    m_blocks.push_back(block);
  }

  /**
   * @brief Determine if a real-valued point lies within the extent of the
   * entity for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot is on top of an entity.
   *
   * @param point The point to check.
   *
   * @return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const argos::CVector2& point) const {
    return xspan(real_loc()).value_within(point.GetX()) &&
        yspan(real_loc()).value_within(point.GetY());
  }

  /**
   * @brief Remove a block from the cache's list of blocks.
   *
   * Does not update the block's location.
   */
  void block_remove(const std::shared_ptr<block>& block);

  /**
   * @brief Get the oldest block in the cache (the one that has been in the
   * cache the longest).
   */
  std::shared_ptr<block> block_get(void) { return m_blocks.front(); }

  std::unique_ptr<base_cache> clone(void) const override;

 private:
  // clang-format off
  static int          m_next_id;


  double                              m_resolution;
  std::vector<std::shared_ptr<block>> m_blocks;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_BASE_CACHE_HPP_ */
