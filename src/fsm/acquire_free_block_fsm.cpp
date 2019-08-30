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

#include "fordyca/controller/block_selector.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/block_acq_validator.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"
#include "fordyca/repr/base_block.hpp"

#include "cosm/robots/footbot/footbot_actuation_subsystem.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_free_block_fsm::acquire_free_block_fsm(
    const controller::block_sel_matrix* const matrix,
    crfootbot::footbot_saa_subsystem* const saa,
    ds::dpo_store* const store,
    std::unique_ptr<fsm::expstrat::foraging_expstrat> exp_behavior)
    : ER_CLIENT_INIT("fordyca.fsm.acquire_free_block"),
      acquire_goal_fsm(
          saa,
          std::move(exp_behavior),
          acquire_goal_fsm::hook_list{
              .acquisition_goal =
                  std::bind(&acquire_free_block_fsm::acq_goal_internal),
              .goal_select =
                  std::bind(&acquire_free_block_fsm::block_select, this),
              .candidates_exist =
                  std::bind(&acquire_free_block_fsm::candidates_exist, this),
              .goal_acquired_cb =
                  std::bind(&acquire_free_block_fsm::block_acquired_cb,
                            this,
                            std::placeholders::_1),
              .explore_term_cb =
                  std::bind(&acquire_free_block_fsm::block_exploration_term_cb,
                            this),
              .goal_valid_cb =
                  std::bind(&acquire_free_block_fsm::block_acq_valid,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2)}),
      mc_matrix(matrix),
      mc_store(store) {}

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
cfmetrics::goal_acq_metrics::goal_type acquire_free_block_fsm::acq_goal_internal(
    void) {
  return cfmetrics::goal_acq_metrics::goal_type(
      foraging_acq_goal::type::ekBLOCK);
} /* acq_goal_internal() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool acquire_free_block_fsm::block_exploration_term_cb(void) const {
  return saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect(
      "block");
} /* block_exploration_term_cb() */

bool acquire_free_block_fsm::block_acquired_cb(bool explore_result) const {
  if (explore_result) {
    ER_ASSERT(block_exploration_term_cb(),
              "No block detected after successful exploration?");
    return true;
  } else {
    if (saa()->sensing()->sensor<chal::sensors::ground_sensor>()->detect(
            "block")) {
      return true;
    }
    ER_WARN("Robot arrived at goal, but no block was detected.");
    return false;
  }
} /* block_acquired_cb() */

boost::optional<cfsm::acquire_goal_fsm::candidate_type> acquire_free_block_fsm::
    block_select(void) const {
  controller::block_selector selector(mc_matrix);

  if (auto best = selector(mc_store->blocks(), saa()->sensing()->position())) {
    return boost::make_optional(acquire_goal_fsm::candidate_type(
        best->ent()->rloc(), kBLOCK_ARRIVAL_TOL, best->ent()->id()));
  } else {
    return boost::optional<acquire_goal_fsm::candidate_type>();
  }
} /* block_select() */

bool acquire_free_block_fsm::candidates_exist(void) const {
  return !mc_store->blocks().empty();
} /* candidates_exist() */

bool acquire_free_block_fsm::block_acq_valid(const rmath::vector2d& loc,
                                             uint id) const {
  return block_acq_validator(&mc_store->blocks(), mc_matrix)(loc, id);
} /* block_acq_valid() */

NS_END(controller, fordyca);
