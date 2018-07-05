/**
 * @file arena_map.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_ARENA_MAP_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_ARENA_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "fordyca/params/depth1/static_cache_params.hpp"
#include "fordyca/representation/arena_cache.hpp"
#include "fordyca/representation/arena_grid.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/nest.hpp"
#include "fordyca/support/block_distributor.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct arena_map_params;
}

NS_START(representation);

class cell2D;
class arena_cache;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class arena_map
 * @ingroup representation
 *
 * @brief The arena map stores a logical representation of the state of the
 * arena. Basically, it combines a 2D grid with sets of objects that populate
 * the grid and move around as the state of the arena changes.
 */
class arena_map : public rcppsw::er::client,
                  public rcppsw::patterns::visitor::visitable_any<arena_map> {
 public:
  using cache_vector = std::vector<std::shared_ptr<arena_cache>>;
  using block_vector = std::vector<std::shared_ptr<block>>;

  explicit arena_map(const struct params::arena_map_params* params);

  /**
   * @brief Get the list of all the blocks currently present in the arena.
   *
   * Some blocks may not be visible on the arena_map, as they are being carried
   * by robots.
   */
  block_vector& blocks(void) { return m_blocks; }

  /**
   * @brief Get the list of all the caches currently present in the arena and
   * active.
   */
  cache_vector& caches(void) { return m_caches; }

  /**
   * @brief Remove a cache from the list of caches.
   *
   * After calling this function the victim cache is no longer active in the
   * arena. However, it is not yet deleted, as it may be needed to gather
   * metrics from (needed to capture block pickup from a cache with only 2
   * blocks).
   *
   * @param victim The cache to remove.
   */
  void cache_remove(const std::shared_ptr<arena_cache>& victim);

  /**
   * @brief Delete all caches that have been previously "removed", but not yet
   * deleted.
   */
  void delete_caches(void);

  void caches_removed_reset(void) { m_caches_removed = 0; }
  void caches_removed(uint b) { m_caches_removed += b; }
  uint caches_removed(void) const { return m_caches_removed; }

  cell2D& access(size_t i, size_t j) { return m_grid.access(i, j); }
  cell2D& access(const rcppsw::math::dcoord2& coord) {
    return access(coord.first, coord.second);
  }

  /**
   * @brief Distribute all blocks in the arena. Resets arena state. Should only
   * be called during (re)-initialization.
   */
  void distribute_all_blocks(void);

  /**
   * @brief Distribute a particular block in the arena, according to whatever
   * policy was specified in the .argos file.
   *
   * @param block The block to distribute.
   */
  void distribute_single_block(std::shared_ptr<block>& block) {
    m_block_distributor.distribute_block(m_grid, block);
  }

  size_t xdsize(void) const { return m_grid.xdsize(); }
  size_t ydsize(void) const { return m_grid.ydsize(); }
  size_t xrsize(void) const { return m_grid.xrsize(); }
  size_t yrsize(void) const { return m_grid.yrsize(); }

  /**
   * @brief (Re)-create the static cache in the arena (depth 1 only).
   */
  void static_cache_create(void);

  bool has_static_cache(void) const { return mc_static_cache_params.enable; }

  /**
   * @brief Get the # of blocks available in the arena.
   */
  size_t n_blocks(void) const { return m_blocks.size(); }

  /**
   * @brief Get the # of caches currently in the arena.
   */
  size_t n_caches(void) const { return m_caches.size(); }

  /**
   * @brief Determine if a robot is currently on top of a block (i.e. if the
   * center of the robot has crossed over into the space occupied by the block
   * extent).
   *
   * While the robots also have their own means of checking if they are on a
   * block or not, there are some false positives, so this function is used as
   * the final arbiter when deciding whether or not to trigger a \ref
   * block_found event or a \ref free_block_pickup event for a particular
   * robot. This happens during initialization when the robot's sensors have not
   * yet been properly initialized by ARGoS.
   *
   * @param pos The position of a robot.
   *
   * @return The ID of the block that the robot is on, or -1 if the robot is not
   * actually on a block.
   */
  int robot_on_block(const argos::CVector2& pos) const;

  /**
   * @brief Determine if a robot is currently on top of a cache (i.e. if the
   * center of the robot has crossed over into the space occupied by the block
   * extent).
   *
   * While the robots also have their own means of checking if they are on a
   * cache or not, there are some false positives, so this function is used as
   * the final arbiter when deciding whether or not to trigger a
   * \ref cache_block_drop or a \ref cached_block_pickup event for a particular
   * robot. This happens during initialization when the robot's sensors have not
   * yet been properly initialized by ARGoS.
   *
   * @param pos The position of a robot.
   *
   * @return The ID of the cache that the robot is on, or -1 if the robot is not
   * actually on a cache.
   */
  int robot_on_cache(const argos::CVector2& pos) const;

  /**
   * @brief Get the subgrid for use in calculating a robot's LOS.
   *
   * @param x X coord of the center of the subgrid.
   * @param y Y coord of the center of the subgrid.
   * @param radius The radius of the subgrid.
   *
   * @return The subgrid.
   */
  rcppsw::ds::grid_view<cell2D*> subgrid(size_t x, size_t y, size_t radius) {
    return m_grid.subcircle(x, y, radius);
  }
  double grid_resolution(void) { return m_grid.resolution(); }
  const representation::nest& nest(void) const { return m_nest; }

 private:
  // clang-format off
  uint                                             m_caches_removed{0};
  const struct params::depth1::static_cache_params mc_static_cache_params;
  block_vector                                     m_blocks;
  cache_vector                                     m_caches;
  support::block_distributor                       m_block_distributor;
  std::shared_ptr<rcppsw::er::server>              m_server;
  arena_grid                                       m_grid;
  representation::nest                             m_nest;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_ARENA_MAP_HPP_ */
