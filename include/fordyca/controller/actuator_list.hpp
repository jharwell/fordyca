/**
 * @file actuator_list.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_ACTUATOR_LIST_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_ACTUATOR_LIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/nsalias.hpp"
#include "rcppsw/robotics/hal/actuators/differential_drive_actuator.hpp"
#include "rcppsw/robotics/hal/actuators/led_actuator.hpp"
#include "rcppsw/robotics/hal/actuators/wifi_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * @struct actuator_list
 * @brief The list of actuators used by the footbots in FORDYCA.
 * @ingroup fordyca controller
 */
struct actuator_list {
  rrhal::actuators::differential_drive_actuator wheels;
  rrhal::actuators::led_actuator leds;
  rrhal::actuators::wifi_actuator wifi;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_ACTUATOR_LIST_HPP_ */
