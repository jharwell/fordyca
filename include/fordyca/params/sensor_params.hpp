/**
 * @file sensor_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_SENSOR_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_SENSOR_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>
#include "rcppsw/common/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct proximity_params
 * @ingroup params
 */
struct proximity_params {
  proximity_params(void)
      : go_straight_angle_range(argos::CRadians(-1.0), argos::CRadians(1.0)) {}

  /*
   * Maximum tolerance for the proximity reading between the robot and the
   * closest obstacle.  The proximity reading is 0 when nothing is detected and
   * grows exponentially to 1 when the obstacle is touching the robot.
   */
  double delta{0.0};

  /* Angle tolerance range to go straight. */
  argos::CRange<argos::CRadians> go_straight_angle_range;
};

/**
 * @struct sensor_params
 * @ingroup params
 */
struct sensor_params : public rcppsw::common::base_params {
  sensor_params(void) : proximity() {}

  struct proximity_params proximity;
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_SENSOR_PARAMS_HPP_ */
