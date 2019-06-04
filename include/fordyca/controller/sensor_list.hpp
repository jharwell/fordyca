/**
 * @file sensor_list.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_SENSOR_LIST_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_SENSOR_LIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/nsalias.hpp"
#include "rcppsw/robotics/hal/sensors/battery_sensor.hpp"
#include "rcppsw/robotics/hal/sensors/ground_sensor.hpp"
#include "rcppsw/robotics/hal/sensors/colored_blob_camera_sensor.hpp"
#include "rcppsw/robotics/hal/sensors/proximity_sensor.hpp"
#include "rcppsw/robotics/hal/sensors/rab_wifi_sensor.hpp"
#include "rcppsw/robotics/hal/sensors/light_sensor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * @struct sensor_list
 * @brief The list of sensors used by the footbots in FORDYCA.
 * @ingroup fordyca controller
 */
struct sensor_list {
  /* clang-format off */
  rrhal::sensors::rab_wifi_sensor            rabs;
  rrhal::sensors::proximity_sensor           proximity;
  rrhal::sensors::colored_blob_camera_sensor blobs;
  rrhal::sensors::light_sensor               light;
  rrhal::sensors::ground_sensor              ground;
  rrhal::sensors::battery_sensor             battery;
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_SENSOR_LIST_HPP_ */
