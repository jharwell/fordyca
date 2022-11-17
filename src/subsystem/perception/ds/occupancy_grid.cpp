/**
 * \file occupancy_grid.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/subsystem/perception/ds/occupancy_grid.hpp"

#include "fordyca/events/cell2D_unknown.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
occupancy_grid::occupancy_grid(const config::mdpo_config* c_config)
    : ER_CLIENT_INIT("fordyca.subsystem.perception.ds.occupancy_grid"),
      stacked_grid2D(rmath::vector2d(0.0, 0.0),
                     c_config->rlos.grid2D.dims,
                     c_config->rlos.grid2D.resolution,
                     c_config->rlos.grid2D.resolution),
      m_pheromone_repeat_deposit(c_config->pheromone.repeat_deposit) {
  ER_INFO("real=(%fx%f), discrete=(%zux%zu), resolution=%f",
          xrsize(),
          yrsize(),
          xdsize(),
          ydsize(),
          resolution().v());

  for (size_t i = 0; i < xdsize(); ++i) {
    for (size_t j = 0; j < ydsize(); ++j) {
      cell_init(i, j, c_config->pheromone.rho);
    } /* for(j..) */
  } /* for(i..) */
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void occupancy_grid::update(void) {
  size_t xmax = xdsize();
  size_t ymax = ydsize();

  for (size_t i = 0; i < xmax; ++i) {
    for (size_t j = 0; j < ymax; ++j) {
      access<kPheromone>(i, j).update();
    } /* for(j..) */
  } /* for(i..) */

  for (size_t i = 0; i < xmax; ++i) {
    for (size_t j = 0; j < ymax; ++j) {
      cell_state_update(i, j);
    } /* for(j..) */
  } /* for(i..) */
} /* update() */

void occupancy_grid::reset(void) {
  size_t xmax = xdsize();
  size_t ymax = ydsize();
  for (size_t i = 0; i < xmax; ++i) {
    for (size_t j = 0; j < ymax; ++j) {
      access<kCell>(i, j).reset();
    } /* for(j..) */
  } /* for(i..) */
} /* Reset */

void occupancy_grid::cell_init(size_t i, size_t j, double pheromone_rho) {
  access<kPheromone>(i, j).rho(pheromone_rho);
  cds::cell2D& cell = access<kCell>(i, j);
  cell.loc(rmath::vector2z(i, j));
} /* cell_init() */

void occupancy_grid::cell_state_update(size_t i, size_t j) {
  crepr::pheromone_density& density = access<kPheromone>(i, j);
  cds::cell2D& cell = access<kCell>(i, j);

  if (!m_pheromone_repeat_deposit) {
    ER_ASSERT(density.v() <= 1.0,
              "Repeat pheromone deposit detected for cell@(%zu, %zu) (%f > "
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
    ER_TRACE("Relevance of cell(%zu, %zu) is within %f of 0", i, j, kEPSILON);
    fevents::cell2D_unknown_visitor op(cell.loc());
    op.visit(*this);
    density.reset();
  }
} /* cell_state_update() */

NS_END(ds, perception, subsystem, fordyca);
