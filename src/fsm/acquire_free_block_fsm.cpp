/**
 * \file acquire_free_block_fsm.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_actuation_subsystem.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem2D.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"

#include "fordyca/controller/block_selector.hpp"
#include "fordyca/ds/dpo_store.hpp"
#include "fordyca/fsm/arrival_tol.hpp"
#include "fordyca/fsm/block_acq_validator.hpp"
#include "fordyca/fsm/expstrat/foraging_expstrat.hpp"
#include "fordyca/fsm/foraging_signal.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
acquire_free_block_fsm::acquire_free_block_fsm(
    const fsm_ro_params* c_params,
    crfootbot::footbot_saa_subsystem2D* saa,
    std::unique_ptr<fsm::expstrat::foraging_expstrat> exp_behavior,
    rmath::rng* rng)
    : ER_CLIENT_INIT("fordyca.fsm.acquire_free_block"),
      acquire_goal_fsm(
          saa,
          std::move(exp_behavior),
          rng,
          acquire_goal_fsm::hook_list{
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  acquisition_goal,
                  std::bind(&acquire_free_block_fsm::acq_goal_internal)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_select,
                  std::bind(&acquire_free_block_fsm::block_select, this)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  candidates_exist,
                  std::bind(&acquire_free_block_fsm::candidates_exist, this)),
              RCPPSW_STRUCT_DOT_INITIALIZER(begin_acq_cb, nullptr),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_acquired_cb,
                  std::bind(&acquire_free_block_fsm::block_acquired_cb,
                            this,
                            std::placeholders::_1)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  explore_term_cb,
                  std::bind(&acquire_free_block_fsm::block_exploration_term_cb,
                            this)),
              RCPPSW_STRUCT_DOT_INITIALIZER(
                  goal_valid_cb,
                  std::bind(&acquire_free_block_fsm::block_acq_valid,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2))}),
      mc_matrix(c_params->bsel_matrix),
      mc_store(c_params->store) {}

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
cfsm::metrics::goal_acq_metrics::goal_type acquire_free_block_fsm::acq_goal_internal(
    void) {
  return fsm::to_goal_type(foraging_acq_goal::ekBLOCK);
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
        best->rloc(), kBLOCK_ARRIVAL_TOL, best->id()));
  } else {
    return boost::optional<acquire_goal_fsm::candidate_type>();
  }
} /* block_select() */

bool acquire_free_block_fsm::candidates_exist(void) const {
  return !mc_store->blocks().empty();
} /* candidates_exist() */

bool acquire_free_block_fsm::block_acq_valid(const rmath::vector2d& loc,
                                             const rtypes::type_uuid& id) const {
  return block_acq_validator(&mc_store->blocks(), mc_matrix)(loc, id);
} /* block_acq_valid() */

NS_END(controller, fordyca);
