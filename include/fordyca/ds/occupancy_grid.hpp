/**
 * @file occupancy_grid.hpp
 * @ingroup ds
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
#include <tuple>

#include "fordyca/ds/cell2D.hpp"
#include "rcppsw/ds/stacked_grid.hpp"
#include "rcppsw/math/dcoord.hpp"
#include "rcppsw/swarm/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct occupancy_grid_params;
}

NS_START(ds);
using robot_layer_stack = std::tuple<rcppsw::swarm::pheromone_density, cell2D>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class occupancy_grid
 * @ingroup ds
 *
 * @brief Multilayered grid of \ref cell2D and \ref rcppsw::swarm::pheromone_density.
 */
class occupancy_grid : public rcppsw::er::client<occupancy_grid>,
                       public rcppsw::ds::stacked_grid<robot_layer_stack> {
 public:
  constexpr static uint kPheromone = 0;
  constexpr static uint kCell = 1;

  occupancy_grid(const struct params::occupancy_grid_params* c_params,
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

 private:
  void cell_update(size_t i, size_t j);
  void cell_init(size_t i, size_t j, double pheromone_rho);

  // clang-format off
  static constexpr double             kEPSILON{0.0001};

  bool                                m_pheromone_repeat_deposit;
  std::string                         m_robot_id;
  // clang-format on
};

NS_END(ds, fordyca);

#endif /* INCLUDE_FORDYCA_DS_OCCUPANCY_GRID_HPP_ */
