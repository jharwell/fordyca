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
#include "fordyca/representation/grid2D.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/support/block_distributor.hpp"
#include "rcppsw/common/er_server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief The arena map stores a logical representation of the state of the
 * arena. Basically, it combines a 2D grid with sets of objects that populate
 * the grid and move around as the state of the arena changes.
 */
class arena_map: public rcppsw::common::er_client {
 public:
  arena_map(const struct grid_params* params,
            argos::CRange<argos::Real> nest_x,
            argos::CRange<argos::Real> nest_y);

  std::vector<block>& blocks(void) { return m_blocks; }
  cell2D& access(size_t i, size_t j) { return m_grid.access(i, j); }
  void distribute_blocks(bool first_time);
  void distribute_block(block& block, bool first_time);
  size_t n_blocks(void) const { return m_blocks.size(); }
  bool respawn_enabled(void) const { return m_block_distributor.respawn_enabled(); }
  int robot_on_block(const argos::CVector2& pos);
  grid_view<cell2D*> subgrid(double x, double y, double radius) {
    return m_grid.subgrid(x, y, radius);
  }
  double grid_resolution(void) { return m_grid.resolution(); }

  /* events */
  void event_block_pickup(block& block, size_t robot_index);
  void event_block_nest_drop(block& block);

 private:
  std::vector<block> m_blocks;
  support::block_distributor m_block_distributor;
  std::shared_ptr<rcppsw::common::er_server> m_server;
  grid2D<cell2D> m_grid;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_ARENA_MAP_HPP_ */
