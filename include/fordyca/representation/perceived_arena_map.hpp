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

#include "rcppsw/ds/grid2D_ptr.hpp"
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/representation/perceived_block.hpp"
#include "fordyca/representation/perceived_cache.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace rcppsw { namespace er { class server; }}

NS_START(fordyca);
namespace params { struct perceived_grid_params; }
NS_START(representation);
class line_of_sight;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class perceived_arena_map
 *
 * @brief The arena map stores a logical representation of the state of the
 * arena. Basically, it combines a 2D grid with sets of objects that populate
 * the grid and move around as the state of the arena changes.
 */
class perceived_arena_map: public rcppsw::er::client,
                           public rcppsw::patterns::visitor::visitable_any<perceived_arena_map> {
 public:
  perceived_arena_map(const std::shared_ptr<rcppsw::er::server>& server,
                      const struct params::perceived_grid_params* params,
                      const std::string& robot_id);

  /**
   * @brief Get a list of all blocks the robot is currently aware of/that are
   * currently relevant.
   *
   * @return The list of perceived blocks (really a list of std::pair<block,
   * double>).
   */
  std::list<perceived_block> blocks(void) const;

  /**
   * @brief Get a list of all caches the robot is currently aware of/that are
   * currently relevant.
   *
   * @return The list of perceived caches (really a list of std::pair<cache,
   * double>).
   */
  std::list<perceived_cache> caches(void) const;

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
  const perceived_cell2D& access(size_t i, size_t j) const { return m_grid.access(i, j); }

  /**
   * @brief Update the density of all cells in the perceived arena.
   */
  void update_density(void);

 private:
  std::shared_ptr<rcppsw::er::server>                          m_server;
  rcppsw::ds::grid2D_ptr<perceived_cell2D,
                         std::shared_ptr<rcppsw::er::server>&> m_grid;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_ARENA_MAP_HPP_ */
