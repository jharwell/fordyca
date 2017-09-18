/**
 * @file perceived_cell2D.hpp
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

#ifndef INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_
#define INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <utility>
#include <string>

#include "rcppsw/swarm/pheromone_density.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Per-robot representation of a cell on the 2D grid. The fsm er_server
 * is disabled, so you can't use any of the standard event reporting macros
 * without modifying \ref grid2D.
 */
class perceived_cell2D : public visitor::visitable<perceived_cell2D>,
                         public rcppsw::common::er_client {
 public:
  explicit perceived_cell2D(
      const std::shared_ptr<rcppsw::common::er_server>& server);

  /**
   * @brief Set the relevance decay parameter for the cell.
   *
   * @param rho The new value.
   */
  void rho(double rho) { m_density.rho(rho); }
  void robot_id(const std::string& robot_id) { m_robot_id = robot_id; }
  const std::string& robot_id(void) { return m_robot_id; }

  /**
   * @brief Get the current information relavence via the current pheromone
   * density of the cell.
   *
   * @return The current relevance.
   */
  double density(void) const { return m_density.last_result(); }

  bool state_is_known(void) { return m_cell.state_is_known(); }
  bool state_has_block(void) { return m_cell.state_has_block(); }
  bool state_has_cache(void) { return m_cell.state_has_cache(); }
  bool state_is_empty(void) { return m_cell.state_is_empty(); }

  /**
   * @brief Get the block current associated with this cell. NULL if no block
   * currently associated.
   *
   * @return The associated block.
   */
  const representation::block* block(void) const { return m_cell.block(); }

  /**
   * @brief Update the information relevance/pheromone density associated with
   * this cell.
   *
   * Every timestep, the relevance decays. Update reaching \ref kEpsilon, the
   * cell transitions back to an unknown state, as the robot can no longer trust
   * its information.
   */
  void update_density(void);

  /**
   * @brief Add the specified amount to the pheromone density for this cell.
   *
   * @param density The amount of pheromone to add.
   */
  void update_density(double density) { m_density.add_pheromone(density); }

  cell2D& cell(void) { return m_cell; }

 private:
  /**
   * The tolerance to zero which the pheromone density has to reach before the
   * cell will transition back to an unknown state.
   */
  static const double kEpsilon;

  std::string m_robot_id;  /// For debugging purposes only
  rcppsw::swarm::pheromone_density m_density;
  cell2D m_cell;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_ */
