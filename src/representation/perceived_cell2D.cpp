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
#include "fordyca/events/cell_unknown.hpp"
#include "fordyca/representation/block.hpp"
#include "fordyca/representation/cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
perceived_cell2D::perceived_cell2D(
    const std::shared_ptr<rcppsw::er::server> &server)
    : decorator(server),
      client(server),
      m_pheromone_repeat_deposit(false),
      m_robot_id(),
      m_density() {
  if (ERROR == attmod("perceived_cell2D")) {
    insmod("perceived_cell2D",
           rcppsw::er::er_lvl::DIAG,
           rcppsw::er::er_lvl::NOM);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perceived_cell2D::density_update(void) {
  if (!pheromone_repeat_deposit()) {
    ER_ASSERT(m_density.last_result() <= 1.0,
              "FATAL: Repeat pheromone deposit detected");
  }

  if (m_density.calc() < kEpsilon) {
    if (decoratee().state_has_block()) {
      ER_VER("Relevance of block%d is within %f of 0 for %s",
             block()->id(),
             kEpsilon,
             m_robot_id.c_str());
    } else if (decoratee().state_has_cache()) {
      ER_VER("Relevance of cache%d is within %f of 0 for %s",
             cache()->id(),
             kEpsilon,
             m_robot_id.c_str());
    }
    events::cell_unknown op(decorator::decoratee().loc().first,
                            decorator::decoratee().loc().second);
    decoratee().accept(op);
  }
} /* density_update() */

NS_END(representation, fordyca);
