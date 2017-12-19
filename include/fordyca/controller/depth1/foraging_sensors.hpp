/**
 * @file depth1_foraging_sensors.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_FORAGING_SENSORS_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_FORAGING_SENSORS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth0/foraging_sensors.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class foraging_sensors
 *
 * @brief The sensors used by the depth1 foraging controller
 */
class foraging_sensors: public depth0::foraging_sensors {
 public:
  foraging_sensors(
      const struct params::sensor_params* c_params,
      argos::CCI_RangeAndBearingSensor* rabs,
      argos::CCI_FootBotProximitySensor* proximity,
      argos::CCI_FootBotLightSensor* light,
      argos::CCI_FootBotMotorGroundSensor* ground);

  foraging_sensors(const foraging_sensors& fsm) = delete;
  foraging_sensors& operator=(const foraging_sensors& fsm) = delete;

  /**
   * @brief If \c TRUE, a cache has *possibly* been detected.
   *
   * Only possibly, because there are some false positives, such as the first
   * timestep, before ARGoS has finished initializing things.
   */
  bool cache_detected(void);
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_FORAGING_SENSORS_HPP_ */
