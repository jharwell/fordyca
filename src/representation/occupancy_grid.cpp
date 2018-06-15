/**
 * @file occupancy_grid.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/occupancy_grid.hpp"
#include "fordyca/events/cell_unknown.hpp"
#include "fordyca/params/occupancy_grid_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
occupancy_grid::occupancy_grid(
    std::shared_ptr<rcppsw::er::server> server,
    const struct params::occupancy_grid_params* c_params,
    const std::string& robot_id)
    : client(),
      stacked_grid2(c_params->grid.resolution,
                    static_cast<size_t>(c_params->grid.upper.GetX()),
                    static_cast<size_t>(c_params->grid.upper.GetY())),
      m_pheromone_repeat_deposit(c_params->pheromone.repeat_deposit),
      m_robot_id(robot_id),
      m_server(std::move(server)) {
  deferred_client_init(m_server);
  insmod("occupancy_grid", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
  ER_NOM("%zu x%zu/%zu x %zu @ %f resolution",
         stacked_grid2::xdsize(),
         stacked_grid2::ydsize(),
         stacked_grid2::xrsize(),
         stacked_grid2::yrsize(),
         stacked_grid2::resolution());

  for (size_t i = 0; i < stacked_grid2::xdsize(); ++i) {
    for (size_t j = 0; j < stacked_grid2::ydsize(); ++j) {
      cell_init(i, j, c_params->pheromone.rho);
    } /* for(j..) */
  }   /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void occupancy_grid::update(void) {
#pragma omp parallel for
  for (size_t i = 0; i < stacked_grid2::xdsize(); ++i) {
    for (size_t j = 0; j < stacked_grid2::ydsize(); ++j) {
      cell_update(i, j);
    } /* for(j..) */
  }   /* for(i..) */
} /* update() */

void occupancy_grid::Reset(void) {
  for (size_t i = 0; i < stacked_grid2::xdsize(); ++i) {
    for (size_t j = 0; j < stacked_grid2::ydsize(); ++j) {
      cell2D& cell = stacked_grid2::access<kCellLayer>(i, j);
      cell.reset();
    } /* for(j..) */
  }   /* for(i..) */
} /* Reset */

void occupancy_grid::cell_init(size_t i, size_t j, double pheromone_rho) {
  stacked_grid2::access<kPheromoneLayer>(i, j).rho(pheromone_rho);
  cell2D& cell = stacked_grid2::access<kCellLayer>(i, j);
  cell.robot_id(m_robot_id);
  cell.loc(rcppsw::math::dcoord2(i, j));
  cell.fsm().deferred_client_init(m_server);
} /* cell_init() */

void occupancy_grid::cell_update(size_t i, size_t j) {
  rcppsw::swarm::pheromone_density& density =
      stacked_grid2::access<kPheromoneLayer>(i, j);
  cell2D& cell = stacked_grid2::access<kCellLayer>(i, j);
  if (!m_pheromone_repeat_deposit) {
    ER_ASSERT(density.last_result() <= 1.0,
              "FATAL: Repeat pheromone deposit detected");
  }

  if (density.calc() < kEPSILON) {
    ER_VER("Relevance of cell(%zu, %zu) is within %f of 0 for %s",
           i,
           j,
           kEPSILON,
           m_robot_id.c_str());
    events::cell_unknown op(cell.loc().first, cell.loc().second);
    cell.accept(op);
  }
} /* cell_update() */

NS_END(representation, fordyca);
