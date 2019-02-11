/**
 * @file ee_max_fsm.hpp
 *
 * @copyright 2019 Anthony Chen/John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_EE_MAX_FSM_HPP_
#define INCLUDE_FORDYCA_FSM_EE_MAX_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/patterns/visitor/visitable.hpp"
#include "fordyca/fsm/base_foraging_fsm.hpp"
#include "fordyca/fsm/explore_for_goal_fsm.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/metrics/fsm/collision_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/controller/sensing_subsystem.hpp"


/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace state_machine = rcppsw::patterns::state_machine;
namespace visitor = rcppsw::patterns::visitor;
namespace controller { class sensing_subsystem; class actuation_subsystem;
                       class ee_decision_matrix; }
namespace ta = rcppsw::task_allocation;

NS_START(fsm);
using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = block_transporter::goal_type;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class ee_max_fsm
 * @ingroup fsm depth0
 *
 * @brief The FSM for the most performing an energy efficient foraging method:
 * each robot takes into consideration the battery left while foraging
 */
class ee_max_fsm : public base_foraging_fsm,
                               public er::client<ee_max_fsm>
                   {
 public:
  explicit ee_max_fsm(const controller::ee_decision_matrix* matrix,
                      controller::saa_subsystem* saa);

  ee_max_fsm(const ee_max_fsm& fsm) = delete;
  ee_max_fsm& operator=(const ee_max_fsm& fsm) = delete;

    /**
   * @brief Run the FSM in its current state, without injecting an event.
   */
  void run(void);

  enum fsm_states {
    ST_START, /* Initial state */
    ST_FORAGING,
    ST_RETREATING,
    ST_CHARGING,
    ST_MAX_STATES,
  };

  void set_taskable(ta::taskable* task) { taskable_fsm = task; }


 private:

  /* ee_max fsm states */
  HFSM_STATE_INHERIT(base_foraging_fsm, transport_to_nest,
                     state_machine::event_data);
  HFSM_ENTRY_INHERIT_ND(base_foraging_fsm, entry_transport_to_nest);

  HFSM_STATE_DECLARE(ee_max_fsm, start, state_machine::event_data);
  HFSM_STATE_DECLARE(ee_max_fsm, foraging, state_machine::event_data);
  HFSM_STATE_DECLARE_ND(ee_max_fsm, charging);


  /**
   * @brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum fsm_states, or things will not work correctly.
   */
  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
  return (&mc_state_map[index]);
  }

  // clang-format off
  ta::taskable* taskable_fsm;
  const controller::ee_decision_matrix* const mc_matrix;
  controller::saa_subsystem* saa;
  // clang-format on

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ST_MAX_STATES);
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_EE_MAX_FSM_HPP_ */
