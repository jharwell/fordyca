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
#include <string>

#include "rcppsw/swarm/pheromone_density.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);
namespace decorator = rcppsw::patterns::decorator;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class perceived_cell2D
 *
 * @brief Per-robot representation of a cell on the 2D grid, as it appears to a
 * robot, in which the knowledge of the cell's state decays over time.
 */
class perceived_cell2D : public decorator::decorator<cell2D>,
                         public visitor::visitable_any<perceived_cell2D>,
                         public rcppsw::er::client {
 public:
  explicit perceived_cell2D(
      const std::shared_ptr<rcppsw::er::server>& server);

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

  bool state_is_known(void) const { return decorator::decoratee().state_is_known(); }
  bool state_has_block(void) const { return decorator::decoratee().state_has_block(); }
  bool state_has_cache(void) const { return decorator::decoratee().state_has_cache(); }
  bool state_is_empty(void) const { return decorator::decoratee().state_is_empty(); }

  size_t block_count(void) const { return decorator::decoratee().block_count(); }

  /**
   * @brief Get the block current associated with this cell. NULL if no block
   * currently associated.
   *
   * @return The associated block.
   */
  const representation::block* block(void) const { return decorator::decoratee().block(); }

  const representation::cache* cache(void) const { return decorator::decoratee().cache(); }

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
   * @param amount The amount of pheromone to add.
   */
  void add_pheromone(double amount) { m_density.add_pheromone(amount); }

  cell2D& cell(void) { return decorator::decoratee(); }

  double epsilon(void) const { return kEpsilon; }

 private:
  /**
   * The tolerance to zero which the pheromone density has to reach before the
   * cell will transition back to an unknown state.
   */
  static const double              kEpsilon;

  std::string                      m_robot_id;
  rcppsw::swarm::pheromone_density m_density;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_ */
