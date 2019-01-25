/**
 * @file occupancy_grid.hpp
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

#ifndef INCLUDE_FORDYCA_DS_OCCUPANCY_GRID_HPP_
#define INCLUDE_FORDYCA_DS_OCCUPANCY_GRID_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <tuple>

#include "fordyca/ds/cell2D.hpp"
#include "rcppsw/ds/stacked_grid.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/swarm/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace events {
class cell_unknown;
class cell_empty;
} // namespace events
namespace params { namespace perception {
struct perception_params;
}} // namespace params::perception

NS_START(ds);
using robot_layer_stack = std::tuple<rcppsw::swarm::pheromone_density, cell2D>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class occupancy_grid
 * @ingroup ds
 *
 * @brief Multilayered grid of cells and associated information
 * density/relevance on the state of those cells. Used by robots in making
 * decisions in how they execute their tasks.
 */
class occupancy_grid : public rcppsw::er::client<occupancy_grid>,
                       public visitor::accept_set<occupancy_grid,
                                                  events::cell_unknown,
                                                  events::cell_empty>,
                       public rcppsw::ds::stacked_grid<robot_layer_stack> {
 public:
  /**
   * @brief The index of the \ref rcppsw::swarm::pheromone_density layer.
   */
  constexpr static uint kPheromone = 0;

  /**
   * @brief The index of the \ref cell2D layer.
   */
  constexpr static uint kCell = 1;

  occupancy_grid(const params::perception::perception_params* c_params,
                 const std::string& robot_id);

  /**
   * @brief Update the density of all cells in the grid.
   */
  void update(void);

  /**
   * @brief Reset all the cells in the grid
   */
  void reset(void);

  bool pheromone_repeat_deposit(void) const {
    return m_pheromone_repeat_deposit;
  }

  uint known_cell_count(void) const { return m_known_cell_count; }
  void known_cells_inc(void) { ++m_known_cell_count; }
  void known_cells_dec(void) { --m_known_cell_count; }

 private:
  /**
   * @brief Update the state of cell (i,j), which involves decreasing its
   * pheromone density, and possibly reseting the cell to be empty if its
   * density gets very close to 0.
   */
  void cell_state_update(uint i, uint j);

  /**
   * @brief Initialize a cell in the occupancy grid, which sets the rate of
   * pheromone decay for the cell, the cell's own reference to its
   * location. Needed because the underlying boost data structure does not
   * support non zero parameter constructors, and we do *NOT* want to use
   * pointers to cells, because that kills our memory performance.
   */
  void cell_init(uint i, uint j, double pheromone_rho);

  /* clang-format off */
  /**
   * @brief The threshold for a cell's pheromone density at which it will
   * transition into back an UNKNOWN state, from whatever state it is currently
   * in.
   */

  static constexpr double             kEPSILON{0.0001};

  uint                                m_known_cell_count{0};
  bool                                m_pheromone_repeat_deposit;
  std::string                         m_robot_id;
  /* clang-format on */
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_OCCUPANCY_GRID_HPP_ */
