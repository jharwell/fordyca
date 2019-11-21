/**
 * @file saa_xml_names.hpp
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

#ifndef INCLUDE_FORDYCA_CONFIG_SAA_XML_NAMES_HPP_
#define INCLUDE_FORDYCA_CONFIG_SAA_XML_NAMES_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * @struct saa_xml_names
 * @ingroup config
 *
 * @brief Collection of names of sensing and actuation component names that
 * ARGoS expects when asking for handles to robot components. Collected here in
 * a single place in the interest of DRY.
 */
struct saa_xml_names {
  static constexpr char diff_steering_saa[] = "differential_steering";
  static constexpr char leds_saa[] = "leds";
  static constexpr char rab_saa[] = "range_and_bearing";
  static constexpr char prox_sensor[] = "footbot_proximity";
  static constexpr char camera_sensor[] = "colored_blob_omnidirectional_camera";
  static constexpr char light_sensor[] = "footbot_light";
  static constexpr char ground_sensor[] = "footbot_motor_ground";
  static constexpr char battery_sensor[] = "battery";
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_SAA_XML_NAMES_HPP_ */
