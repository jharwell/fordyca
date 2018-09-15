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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include "fordyca/representation/occupancy_grid.hpp"
#include "fordyca/representation/perceived_block.hpp"
#include "fordyca/representation/perceived_cache.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace params {
struct occupancy_grid_params;
}

NS_START(representation);
class line_of_sight;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class perceived_arena_map
 * @ingroup representation
 *
 * @brief The arena map stores a logical representation of the state of the
 * arena, from the perspective of the robot.
 *
 * Crucially, this class stores the caches SEPARATELY from the \ref arena_map
 * where they actually live (clone not reference), which decouples/simplifies a
 * lot of the tricky handshaking logic for picking up/dropping blocks in caches.
 */
class perceived_arena_map
    : public rcppsw::er::client<perceived_arena_map>,
      public rcppsw::patterns::visitor::visitable_any<perceived_arena_map> {
 public:
  using cache_list = std::list<std::shared_ptr<base_cache>>;
  using block_list = std::list<std::shared_ptr<base_block>>;
  using perceived_cache_list = std::list<perceived_cache>;
  using perceived_block_list = std::list<perceived_block>;

  perceived_arena_map(
      const struct fordyca::params::occupancy_grid_params* c_params,
      const std::string& robot_id);

  bool pheromone_repeat_deposit(void) const {
    return m_grid.pheromone_repeat_deposit();
  }

  /**
   * @brief Get a list of all blocks the robot is currently aware of and their
   * relevance.
   *
   * @return The list of perceived blocks.
   */
  perceived_block_list perceived_blocks(void) const;

  /**
   * @brief Get a list of all blocks the robot is currently aware of.
   */
  block_list& blocks(void) { return m_blocks; }

  /**
   * @brief Get a list of all cache the robot is currently aware of and their
   * relevance.
   *
   * @return The list of perceived cache.
   */
  perceived_cache_list perceived_caches(void) const;

  /**
   * @brief Get a list of all caches the robot is currently aware of.
   */
  cache_list& caches(void) { return m_caches; }

  /**
   * @brief Add a cache to the list of perceived caches. If there is already a
   * known cache at that location, it is removed and replaced with the new one.
   *
   * @param cache Cache to add.
   */
  void cache_add(const std::shared_ptr<base_cache>& cache);

  /**
   * @brief Remove a cache from the list of perceived caches, and update its
   * cell to be empty.
   */
  void cache_remove(const std::shared_ptr<base_cache>& victim);

  /*
   * @brief Add a free block to the list of known blocks.
   *
   * If the block is already in our list of blocks we know about it needs to be
   * removed, because the new version we just got from our LOS is more up to
   * date.
   */
  bool block_add(const std::shared_ptr<base_block>& block);

  /*
   * @brief Remove a block from the list of known blocks, and update its cell to
   * be empty.
   */
  bool block_remove(const std::shared_ptr<base_block>& victim);

  /**
   * @brief Access a particular element in the discretized grid representing the
   * robot's view of the arena. No bounds checking is performed, so if something
   * is out of bounds, boost with fail with a bounds checking assertion.
   *
   * @param i X coord.
   * @param j Y coord
   *
   * @return The cell.
   */
  template <int Index>
  typename occupancy_grid::layer_value_type<Index>::value_type& access(size_t i,
                                                                       size_t j) {
    return m_grid.access<Index>(i, j);
  }
  template <int Index>
  const typename occupancy_grid::layer_value_type<Index>::value_type& access(
      size_t i,
      size_t j) const {
    return m_grid.access<Index>(i, j);
  }
  template <int Index>
  typename occupancy_grid::layer_value_type<Index>::value_type& access(
      const rcppsw::math::dcoord2& d) {
    return m_grid.access<Index>(d);
  }
  template <int Index>
  const typename occupancy_grid::layer_value_type<Index>::value_type& access(
      const rcppsw::math::dcoord2& d) const {
    return m_grid.access<Index>(d);
  }

  /**
   * @brief Update the density of all cells in the perceived arena.
   */
  void update(void) { m_grid.update(); }

  /**
   * @brief Reset all the cells in the percieved arena.
   */
  void reset(void) { m_grid.reset(); }

  double grid_resolution(void) const { return m_grid.resolution(); }

 private:
  // clang-format off
  occupancy_grid                      m_grid;
  // clang-format on

  /**
   * @brief The caches that the robot currently knows about. Their relevance is
   * not stored with the cache, because that is a properly of the cell the cache
   * resides in, and not the cache itself. These are pointers, rather than a
   * contiguous array, to get better support from valgrind for debugging.
   */
  cache_list m_caches;

  /**
   * @brief The blocks that the robot currently knows about. Their relevance is
   * not stored with the block, because that is a properly of the cell the block
   * resides in, and not the block itself.These are pointers, rather than a
   * contiguous array, to get better support from valgrind for debugging.
   */
  block_list m_blocks;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_ */
