/**
 * @file perceived_cell2D.cpp
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
#include "fordyca/representation/perceived_cell2D.hpp"
#include "fordyca/events/block_drop.hpp"
#include "fordyca/events/cell_empty.hpp"
#include "fordyca/events/cell_unknown.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constants
 ******************************************************************************/
const double perceived_cell2D::kEpsilon = 0.0001;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perceived_cell2D::update_density(void) {
  if (m_cell.state_is_known()) {
    m_density.calc();
    if (m_density.last_result() < kEpsilon) {
      events::cell_unknown op;
      m_cell.accept(op);
    }
  }
} /* update_density() */

NS_END(representation, fordyca);
