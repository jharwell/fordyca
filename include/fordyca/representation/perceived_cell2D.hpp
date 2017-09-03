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
#include "rcppsw/common/common.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/expressions/expressions.hpp"

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
class perceived_cell2D  {
 public:
  explicit perceived_cell2D(
      const std::shared_ptr<rcppsw::common::er_server>& server) :
      m_rho(0.0), m_density(), m_cell(server) {}

  void rho(double rho) { m_density.rho(rho); }
  double density(void) const { return m_density.last_result(); }

  bool state_is_known(void) { return m_cell.state_is_known(); }
  bool state_has_block(void) { return m_cell.state_has_block(); }
  bool state_is_empty(void) { return m_cell.state_is_empty(); }
  const representation::block* block(void) const { return m_cell.block(); }

  /**
   * @brief Update the relevance/freshness of the information about the state of
   * the current square.
   *
   * Each call to this function increases the relevance of the information by a
   * factor of 1.0.
   */
  void update_relevance(void);
  void event_encounter(cell2D_fsm::state state,
                       representation::block* block = nullptr);

 private:
  static const double kEpsilon;
  double m_rho;
  expressions::pheromone_density m_density;
  cell2D m_cell;
};

NS_END(representation, fordyca);

#endif /* INCLUDE_FORDYCA_REPRESENTATION_PERCEIVED_CELL2D_HPP_ */
