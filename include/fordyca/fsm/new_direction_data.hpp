/**
 * @file new_direction_data.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_FSM_NEW_DIRECTION_DATA_HPP_
#define INCLUDE_FORDYCA_FSM_NEW_DIRECTION_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/state_machine/event.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace rfsm = rcppsw::patterns::state_machine;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @struct new_direction_data
 * @ingroup fsm
 *
 * @brief Structure containing randomness to inject into robot motion by having
 * them change their direction.
 */
struct new_direction_data : public rfsm::event_data {
  explicit new_direction_data(const rmath::radians& dir_in) : dir(dir_in) {}

  rmath::radians dir;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_NEW_DIRECTION_DATA_HPP_ */
