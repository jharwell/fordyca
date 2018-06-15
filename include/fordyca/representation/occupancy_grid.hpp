/**
 * @file occupancy_grid.hpp
 * @ingroup representation
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_OCCUPANCY_GRID_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_OCCUPANCY_GRID_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <tuple>

#include "fordyca/representation/cell2D.hpp"
#include "rcppsw/ds/stacked_grid.hpp"
#include "rcppsw/math/dcoord.hpp"
#include "rcppsw/swarm/pheromone_density.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct occupancy_grid_params; }

NS_START(representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using layer_stack = std::tuple<rcppsw::swarm::pheromone_density, cell2D>;

class occupancy_grid : public rcppsw::er::client,
                       public rcppsw::ds::stacked_grid2<layer_stack> {
 public:
  occupancy_grid(std::shared_ptr<rcppsw::er::server> server,
                 const struct params::occupancy_grid_params* c_params,
                 const std::string& robot_id);

  /**
   * @brief Update the density of all cells in the grid.
   */
  void update(void);

  /**
   * @brief Reset all the cells in the grid
   */
  void Reset(void);

  bool pheromone_repeat_deposit(void) const {
    return m_pheromone_repeat_deposit;
  }
  constexpr static uint kPheromoneLayer = 0;
  constexpr static uint kCellLayer = 1;

 private:
  void cell_update(size_t i, size_t j);
  void cell_init(size_t i, size_t j, double pheromone_rho);

  // clang-format off
  bool                                m_pheromone_repeat_deposit;
  std::string                         m_robot_id;
  static constexpr double             kEPSILON{0.0001};
  std::shared_ptr<rcppsw::er::server> m_server;
  // clang-format on
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_OCCUPANCY_GRID_HPP_ */
