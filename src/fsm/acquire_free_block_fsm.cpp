/**
 * @file acquire_free_block_fsm.cpp
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
#include "fordyca/fsm/acquire_free_block_fsm.hpp"

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/block_selector.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/representation/base_block.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_free_block_fsm::acquire_free_block_fsm(
    const controller::block_sel_matrix* const sel_matrix,
    controller::saa_subsystem* const saa,
    ds::perceived_arena_map* const map)
    : ER_CLIENT_INIT("fordyca.fsm.acquire_free_block"),
      acquire_goal_fsm(
          saa,
          std::bind(&acquire_free_block_fsm::acquisition_goal_internal, this),
          std::bind(&acquire_free_block_fsm::candidates_exist, this),
          std::bind(&acquire_free_block_fsm::block_select, this),
          std::bind(&acquire_free_block_fsm::block_acquired_cb,
                    this,
                    std::placeholders::_1),
          std::bind(&acquire_free_block_fsm::block_exploration_term_cb, this)),
      mc_matrix(sel_matrix),
      mc_map(map) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool acquire_free_block_fsm::block_exploration_term_cb(void) const {
  return saa_subsystem()->sensing()->block_detected();
} /* block_exploration_term_cb() */

bool acquire_free_block_fsm::block_acquired_cb(bool explore_result) const {
  if (explore_result) {
    ER_ASSERT(block_exploration_term_cb(),
              "No block detected after successful exploration?");
    return true;
  } else {
    if (saa_subsystem()->sensing()->block_detected()) {
      return true;
    }
    ER_WARN("Robot arrived at goal, but no block was detected.");
    return false;
  }
} /* block_acquired_cb() */

acquire_goal_fsm::candidate_type acquire_free_block_fsm::block_select(void) const {
  controller::block_selector selector(mc_matrix);
  auto best = selector.calc_best(mc_map->perceived_blocks(),
                                 saa_subsystem()->sensing()->position());
  if (nullptr == best.ent) {
    return acquire_goal_fsm::candidate_type(false, rmath::vector2d(), -1);
  } else {
    return acquire_goal_fsm::candidate_type(true,
                                            best.ent->real_loc(),
                                            vector_fsm::kBLOCK_ARRIVAL_TOL);
  }
} /* block_select() */

bool acquire_free_block_fsm::candidates_exist(void) const {
  return !mc_map->perceived_blocks().empty();
} /* candidates_exsti() */

__rcsw_const acquisition_goal_type
acquire_free_block_fsm::acquisition_goal_internal(void) const {
  return acquisition_goal_type::kBlock;
} /* acquisition_goal() */

NS_END(controller, fordyca);
