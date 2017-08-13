/**
 * @file cell2D.cpp
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
#include "fordyca/representation/cell2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cell2D::update_relevance(void) {
  if (m_fsm.is_known()) {
    m_relevance = std::max(0.0, m_relevance - m_delta);
    if (m_relevance <= 0.0) {
      m_fsm.event_encounter(new cell2D_fsm::encounter_data(
          cell2D_fsm::UNKNOWN));
    }
  }
} /* update_relevance() */

void cell2D::encounter(cell2D_fsm::encounter_type type,
                        int cache_blocks) {
  m_fsm.event_encounter(new cell2D_fsm::encounter_data(type,
                                                           cache_blocks));
  m_relevance += 1.0;
} /* encounter() */

void cell2D::remote_encounter(cell2D_fsm::encounter_type type,
                                  int cache_blocks) {
  if ((m_fsm.is_empty() && type == cell2D_fsm::EMPTY) ||
      (m_fsm.has_block() && type == cell2D_fsm::BLOCK) ||
      (m_fsm.has_cache() && type == cell2D_fsm::CACHE)) {
    encounter(type, cache_blocks);
  } else {
    m_relevance = 0.0;
  }
  encounter(type, cache_blocks);
} /* remote_encounter() */


NS_END(representation, fordyca);
