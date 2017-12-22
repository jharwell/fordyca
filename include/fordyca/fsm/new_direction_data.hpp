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
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/patterns/state_machine/event.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, fsm);
namespace state_machine = rcppsw::patterns::state_machine;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @brief Inject randomness into robot motion by having them change their
 * direction
 */
struct new_direction_data : public state_machine::event_data {
  explicit new_direction_data(argos::CRadians dir_) : dir(dir_) {}

  argos::CRadians dir;
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_NEW_DIRECTION_DATA_HPP_ */
