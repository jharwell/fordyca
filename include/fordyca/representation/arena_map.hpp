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

#include "rcppsw/common/er_client.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/representation/grid2D.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/support/block_distributor.hpp"
#include "fordyca/params/cache_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct arena_map_params; }

NS_START(representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The arena map stores a logical representation of the state of the
 * arena. Basically, it combines a 2D grid with sets of objects that populate
 * the grid and move around as the state of the arena changes.
 */
class arena_map: public rcppsw::common::er_client,
                 public rcppsw::patterns::visitor::visitable<arena_map> {
 public:
  arena_map(const struct params::arena_map_params* params);

  /**
   * @brief Get the list of all the blocks currently present in the arena.
   *
   * Some blocks may not be visible on the arena_map, as they are being carried
   * by robots.
   */
  std::vector<block>& blocks(void) { return m_blocks; }

  std::vector<cache>& caches(void) { return m_caches; }
  cell2D& access(size_t i, size_t j) { return m_grid.access(i, j); }

  /**
   * @brief Distribute all blocks in the arena.
   *
   * @param first_time Is this the first time we are distributing blocks?
   * (needed for simulations in which the blocks do not respawn/reappear after
   * being brought to the nest).
   */
  void distribute_blocks(bool first_time);

  /**
   * @brief Distribute a particular block in the arena, according to whatever
   * policy was specified in the .argos file.
   *
   * @param block The block to distribute.
   * @param first_time Is this the first time distributing this block?
   */
  void distribute_block(block* const block, bool first_time);

  /*
   * @todo: Add a consolidate_blocks() function for optionally running after every block
   * spawning to combine blocks that are close together into caches
   */

  /**
   * @brief Get the # of blocks available in the arena.
   */
  size_t n_blocks(void) const { return m_blocks.size(); }

  /**
   * @brief Get the # of caches currently in the arena.
   */
  size_t n_caches(void) const { return m_caches.size(); }

  /**
   * @brief Check if FORDYCA was configured to have respawning blocks or not. If
   * not, then after a block is brought to the nest and deposited for the first
   * time, then it does not reappear in the arena. In that case, once all blocks
   * have been collected, the simulation ends. Otherwise, the simulation goes on
   * indefinitely.
   *
   * @return TRUE if the condition is met, FALSE otherwise.
   */
  bool respawn_enabled(void) const {
    return m_block_distributor.respawn_enabled();
  }

  /**
   * @brief Determine if a robot is currently on top of a block (i.e. if the
   * center of the robot has crossed over into the space occupied by the block
   * extent).
   *
   * While the robots also have their own means of checking if they are on a
   * block or not, there are some false positives, so this function is used as
   * the final arbiter when deciding whether or not to trigger a "block_found"
   * event for a particular robot. This happens during initialization when the
   * robot's sensors have not yet been properly initialized by ARGoS.
   *
   * @param pos The position of a robot.
   *
   * @return The ID of the block that the robot is on, or -1 if the robot is not
   * actually on a block.
   */
  int robot_on_block(const argos::CVector2& pos);

  /**
   * @brief Get the subgrid for use in calculating a robot's LOS.
   *
   * @param x X coord of the center of the subgrid.
   * @param y Y coord of the center of the subgrid.
   * @param radius The radius of the subgrid.
   *
   * @return The subgrid.
   */
  grid_view<cell2D*> subgrid(size_t x, size_t y, size_t radius) {
    return m_grid.subgrid(x, y, radius);
  }
  double grid_resolution(void) { return m_grid.resolution(); }

 private:
  const struct params::cache_params mc_cache_params;
  const argos::CVector2 mc_nest_center;
  std::vector<block> m_blocks;
  std::vector<cache> m_caches;
  support::block_distributor m_block_distributor;
  std::shared_ptr<rcppsw::common::er_server> m_server;
  grid2D<cell2D> m_grid;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_ARENA_MAP_HPP_ */
