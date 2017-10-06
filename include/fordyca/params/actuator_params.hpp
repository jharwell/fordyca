/**
 * @file actuator_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_ACTUATOR_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_ACTUATOR_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/angles.h>
#include "fordyca/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct wheel_params {
  /* Angular thresholds to change turning state. */
  argos::CRadians hard_turn_threshold;
  argos::CRadians soft_turn_threshold;
  argos::CRadians no_turn_threshold;
  argos::Real max_speed;

  wheel_params(void) :
      hard_turn_threshold(),
      soft_turn_threshold(),
      no_turn_threshold(),
      max_speed() {}
};


struct actuator_params : public base_params {
  actuator_params(void) : wheels() {}

  struct wheel_params wheels;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_ACTUATOR_PARAMS_HPP_ */