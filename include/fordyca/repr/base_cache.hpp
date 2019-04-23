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

#ifndef INCLUDE_FORDYCA_REPR_BASE_CACHE_HPP_
#define INCLUDE_FORDYCA_REPR_BASE_CACHE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <utility>
#include <vector>

#include "fordyca/ds/block_vector.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/repr/immovable_cell_entity.hpp"
#include "fordyca/repr/multicell_entity.hpp"
#include "rcppsw/patterns/prototype/clonable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, repr);
namespace prototype = rcppsw::patterns::prototype;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_cache
 * @ingroup fordyca repr
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
  static constexpr size_t kMinBlocks = 2;

  /**
   * @param dimension The size of the cache. Does not have to be a multiple of
   * the arena resolution, but doing so makes it easier.
   * @param resolution The arena resolution.
   * @param center (X,Y) coordinates of the center of the cache.
   * @param blocks The initial block list for the cache.

   * @param id The ID to assign to the cache; -1 for a new cache, which
   * will generate a new ID, or any positive # to use the same ID as an existing
   * cache (used when cloning a cache into a robot's perception).
   */
  base_cache(double dimension,
             double resolution,
             const rmath::vector2d& center,
             const ds::block_vector& blocks,
             int id);
  ~base_cache(void) override = default;

  /**
   * @brief Disallow direct object comparisons, because we may want to compare
   * for equality in terms of IDs or object locations, and it is better to
   * require explicit comparisons for BOTH, rather than just one. It also makes
   * it unecessary to have to remember which type the comparison operator==()
   * does for this class.
   */
  bool operator==(const base_cache& other) const = delete;

  /**
   * @brief Compare two \ref base_cache objects for equality based on their ID.
   */
  bool idcmp(const base_cache& other) const { return this->id() == other.id(); }

  /**
   * @brief Compare two \ref base_cache objects for equality based on their
   * discrete location.
   */
  bool loccmp(const base_cache& other) const {
    return this->discrete_loc() == other.discrete_loc();
  }

  /**
   * @brief \c TRUE iff the cache contains the specified block.
   */
  __rcsw_pure bool contains_block(
      const std::shared_ptr<base_block>& c_block) const {
    return std::find(m_blocks.begin(), m_blocks.end(), c_block) !=
           m_blocks.end();
  }
  virtual size_t n_blocks(void) const { return blocks().size(); }

  /**
   * @brief Get a list of the blocks currently in the cache.
   */
  ds::block_vector& blocks(void) { return m_blocks; }
  const ds::block_vector& blocks(void) const { return m_blocks; }

  /**
   * @brief Add a new block to the cache's list of blocks.
   *
   * Does not update the block's location.
   */
  void block_add(const std::shared_ptr<base_block>& block) {
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
  bool contains_point(const rmath::vector2d& point) const {
    return xspan(real_loc()).contains(point.x()) &&
           yspan(real_loc()).contains(point.y());
  }

  /**
   * @brief Remove a block from the cache's list of blocks.
   *
   * Does not update the block's location.
   */
  void block_remove(const std::shared_ptr<base_block>& block);

  /**
   * @brief Get the oldest block in the cache (the one that has been in the
   * cache the longest).
   */
  std::shared_ptr<base_block> oldest_block(void) { return m_blocks.front(); }

  std::unique_ptr<base_cache> clone(void) const override;

  uint creation_ts(void) const { return m_creation_ts; }
  void creation_ts(uint creation_ts) { m_creation_ts = creation_ts; }

 private:
  /* clang-format off */
  static int       m_next_id;

  const double     mc_resolution;

  uint             m_creation_ts{0};
  ds::block_vector m_blocks;
  /* clang-format on */
};

NS_END(repr, fordyca);

#endif /* INCLUDE_FORDYCA_REPRSENTATION_BASE_CACHE_HPP_ */
