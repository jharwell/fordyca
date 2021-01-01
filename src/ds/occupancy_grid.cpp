/**
 * \file occupancy_grid.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/ds/occupancy_grid.hpp"

#include "fordyca/events/cell2D_unknown.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
occupancy_grid::occupancy_grid(const cspconfig::perception_config* c_config,
                               const std::string& robot_id)
    : ER_CLIENT_INIT("fordyca.ds.occupancy_grid"),
      stacked_grid2D(rmath::vector2d(0.0, 0.0),
                     c_config->occupancy_grid.dims,
                     c_config->occupancy_grid.resolution,
                     c_config->occupancy_grid.resolution),
      m_pheromone_repeat_deposit(c_config->pheromone.repeat_deposit),
      m_robot_id(robot_id) {
  ER_INFO("real=(%fx%f), discrete=(%zux%zu), resolution=%f",
          xrsize(),
          yrsize(),
          xdsize(),
          ydsize(),
          resolution().v());

  for (uint i = 0; i < xdsize(); ++i) {
    for (uint j = 0; j < ydsize(); ++j) {
      cell_init(i, j, c_config->pheromone.rho);
    } /* for(j..) */
  } /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void occupancy_grid::update(void) {
  uint xmax = xdsize();
  uint ymax = ydsize();

  for (uint i = 0; i < xmax; ++i) {
    for (uint j = 0; j < ymax; ++j) {
      access<kPheromone>(i, j).update();
    } /* for(j..) */
  } /* for(i..) */

  for (uint i = 0; i < xmax; ++i) {
    for (uint j = 0; j < ymax; ++j) {
      cell_state_update(i, j);
    } /* for(j..) */
  } /* for(i..) */
} /* update() */

void occupancy_grid::reset(void) {
  uint xmax = xdsize();
  uint ymax = ydsize();
  for (uint i = 0; i < xmax; ++i) {
    for (uint j = 0; j < ymax; ++j) {
      access<kCell>(i, j).reset();
    } /* for(j..) */
  } /* for(i..) */
} /* Reset */

void occupancy_grid::cell_init(uint i, uint j, double pheromone_rho) {
  access<kPheromone>(i, j).rho(pheromone_rho);
  cds::cell2D& cell = access<kCell>(i, j);
  cell.loc(rmath::vector2z(i, j));
} /* cell_init() */

void occupancy_grid::cell_state_update(uint i, uint j) {
  crepr::pheromone_density& density = access<kPheromone>(i, j);
  cds::cell2D& cell = access<kCell>(i, j);

  if (!m_pheromone_repeat_deposit) {
    ER_ASSERT(density.v() <= 1.0,
              "Repeat pheromone deposit detected for cell@(%u, %u) (%f > "
              "1.0, state=%d)",
              i,
              j,
              density.v(),
              cell.fsm().current_state());
  }

  /*
   * If the cell state is already unknown, don't resend the event. Doesn't
   * matter for just a few robots, but it does when you have hundreds. We could
   * also check if the cell state is known, but that is slower than checking if
   * the density has already been reset.
   */
  if (density.v() < kEPSILON &&
      density.v() > std::numeric_limits<double>::min()) {
    ER_TRACE("Relevance of cell(%u, %u) is within %f of 0 for %s",
             i,
             j,
             kEPSILON,
             m_robot_id.c_str());
    events::cell2D_unknown_visitor op(cell.loc());
    op.visit(*this);
    density.reset();
  }
} /* cell_state_update() */

NS_END(ds, fordyca);
