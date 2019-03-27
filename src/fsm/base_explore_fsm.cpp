/**
 * @file base_explore_fsm.cpp
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
#include "fordyca/fsm/base_explore_fsm.hpp"
#include "fordyca/controller/foraging_signal.hpp"
#include "fordyca/controller/saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);

namespace utils = rcppsw::utils;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_explore_fsm::base_explore_fsm(controller::saa_subsystem* const saa,
                                   uint8_t max_states)
    : base_foraging_fsm(saa, max_states), entry_explore() {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_ENTRY_DEFINE_ND(base_explore_fsm, entry_explore) {
  base_foraging_fsm::actuators()->leds_set_color(utils::color::kMAGENTA);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void base_explore_fsm::task_execute(void) {
  inject_event(controller::foraging_signal::FSM_RUN, rfsm::event_type::NORMAL);
} /* task_execute() */

NS_END(fsm, fordyca);
