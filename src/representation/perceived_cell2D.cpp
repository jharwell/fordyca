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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, representation);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perceived_cell2D::update_relevance(void) {
  if (fsm().is_known()) {
    m_relevance = std::max(0.0, m_relevance - m_delta);
    if (m_relevance <= 0.0) {
      fsm().change_state(new encounter_data(cell2D_fsm::UNKNOWN));
    }
  }
} /* update_relevance() */

void perceived_cell2D::encounter(cell2D_fsm::new_state state,
                        int cache_blocks) {
  fsm().change_state(new encounter_data(state, cache_blocks));
  m_relevance += 1.0;
} /* encounter() */

void perceived_cell2D::remote_encounter(cell2D_fsm::new_state state,
                                  int cache_blocks) {
  if ((fsm().is_empty() && state == cell2D_fsm::EMPTY) ||
      (fsm().has_block() && state == cell2D_fsm::BLOCK) ||
      (fsm().has_cache() && state == cell2D_fsm::CACHE)) {
    encounter(state, cache_blocks);
  } else {
    m_relevance = 0.0;
  }
  encounter(state, cache_blocks);
} /* remote_encounter() */


NS_END(representation, fordyca);
