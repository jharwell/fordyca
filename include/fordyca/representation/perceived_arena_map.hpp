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
#include <string>

#include "fordyca/representation/occupancy_grid.hpp"
#include "fordyca/representation/perceived_block.hpp"
#include "fordyca/representation/perceived_cache.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace rcppsw {
namespace er {
class server;
}
}

NS_START(fordyca);
namespace params {
namespace depth0 {
struct perceived_arena_map_params;
}
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
    : public rcppsw::er::client,
      public rcppsw::patterns::visitor::visitable_any<perceived_arena_map> {
 public:
  perceived_arena_map(
      std::shared_ptr<rcppsw::er::server> server,
      const struct params::depth0::perceived_arena_map_params* c_params,
      const std::string& robot_id);

  /**
   * @brief Get a list of all blocks the robot is currently aware of and their
   * relevance.
   *
   * @return The list of perceived blocks.
   */
  std::list<const_perceived_block> perceived_blocks(void) const;

  /**
   * @brief Get a list of all blocks the robot is currently aware of.
   */
  std::list<std::unique_ptr<block>>& blocks(void) { return m_blocks; }

  /**
   * @brief Get a list of all cache the robot is currently aware of and their
   * relevance.
   *
   * @return The list of perceived cache.
   */
  std::list<const_perceived_cache> perceived_caches(void) const;

  /**
   * @brief Get a list of all caches the robot is currently aware of.
   */
  std::list<std::unique_ptr<representation::cache>>& caches(void) {
    return m_caches;
  }

  /**
   * @brief Add a cache to the list of perceived caches.
   *
   * @param cache Cache to add.
   */
  void cache_add(std::unique_ptr<representation::cache>& cache);

  /**
   * @brief Remove a cache from the list of perceived caches, and update its
   * cell to be empty.
   */
  void cache_remove(const cache* victim);

  /*
   * @brief Add a free block to the list of known blocks.
   *
   * If the block is already in our list of blocks we know about it needs to be
   * removed, because the new version we just got from our LOS is more up to
   * date.
   */
  bool block_add(std::unique_ptr<representation::block>& block);

  /*
   * @brief Remove a block from the list of known blocks, and update its cell to
   * be empty.
   */
  bool block_remove(const block* victim);

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
  perceived_cell2D& access(size_t i, size_t j) { return m_grid.access(i, j); }
  perceived_cell2D& access(const discrete_coord& c) {
    return access(c.first, c.second);
  }
  const perceived_cell2D& access(size_t i, size_t j) const {
    return m_grid.access(i, j);
  }

  /**
   * @brief Update the density of all cells in the perceived arena.
   */
  void update_density(void);

 private:
  // clang-format off
  std::shared_ptr<rcppsw::er::server> m_server;
  perceived_occupancy_grid            m_grid;
  // clang-format on

  /**
   * @brief The caches that the robot currently knows about. Their relevance is
   * not stored with the cache, because that is a properly of the cell the cache
   * resides in, and not the cache itself. These are pointers, rather than a
   * contiguous array, to get better support from valgrind for debugging.
   */
  std::list<std::unique_ptr<representation::cache>> m_caches;

  /**
   * @brief The blocks that the robot currently knows about. Their relevance is
   * not stored with the block, because that is a properly of the cell the block
   * resides in, and not the block itself.These are pointers, rather than a
   * contiguous array, to get better support from valgrind for debugging.
   */
  std::list<std::unique_ptr<representation::block>> m_blocks;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_ */
