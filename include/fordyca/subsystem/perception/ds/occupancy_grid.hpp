/**
 * \file occupancy_grid.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <tuple>

#include "rcppsw/ds/stacked_grid2D.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/pheromone_density.hpp"

#include "fordyca/subsystem/perception/config/mdpo_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, subsystem, perception, ds);

/**
 * \brief The types of layers used by \ref occupancy_grid (i.e. a heterogeneous
 * 3D grid).
 */
using robot_layer_stack = std::tuple<crepr::pheromone_density, cds::cell2D>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class occupancy_grid
 * \ingroup subsystem perception ds
 *
 * \brief Multilayered grid of cells and associated information
 * density/relevance on the state of those cells. Used by robots in making
 * decisions in how they execute their tasks.
 */
class occupancy_grid : public rer::client<occupancy_grid>,
                       public rds::stacked_grid2D<robot_layer_stack> {
 public:
  /**
   * \brief The index of the \ref crepr::pheromone_density layer.
   */
  static constexpr size_t kPheromone = 0;

  /**
   * \brief The index of the \ref cell2D layer.
   */
  static constexpr size_t kCell = 1;

  explicit occupancy_grid(const config::mdpo_config* c_config);

  /**
   * \brief Update the density of all cells in the grid.
   */
  void update(void);

  /**
   * \brief Reset all the cells in the grid
   */
  void reset(void);

  bool pheromone_repeat_deposit(void) const { return m_pheromone_repeat_deposit; }

  size_t known_cell_count(void) const { return m_known_cell_count; }
  void known_cells_inc(void) { ++m_known_cell_count; }
  void known_cells_dec(void) { --m_known_cell_count; }

 private:
  /**
   * \brief Update the state of cell (i,j), which involves decreasing its
   * pheromone density, and possibly reseting the cell to be empty if its
   * density gets very close to 0.
   */
  void cell_state_update(size_t i, size_t j);

  /**
   * \brief Initialize a cell in the occupancy grid, which sets the rate of
   * pheromone decay for the cell, the cell's own reference to its
   * location. Needed because the underlying boost data structure does not
   * support non zero parameter constructors, and we do *NOT* want to use
   * pointers to cells, because that kills our memory performance.
   */
  void cell_init(size_t i, size_t j, double pheromone_rho);

  /* clang-format off */
  /**
   * \brief The threshold for a cell's pheromone density at which it will
   * transition into back an UNKNOWN state, from whatever state it is currently
   * in.
   */

  static constexpr double             kEPSILON{0.0001};

  size_t                              m_known_cell_count{0};
  bool                                m_pheromone_repeat_deposit;
  /* clang-format on */
};

NS_END(ds, perception, subsystem, fordyca);

