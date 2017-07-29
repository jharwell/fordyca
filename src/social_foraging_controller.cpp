/**
 * @file social_foraging_controller.cpp
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
#include "fordyca/social_foraging_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
social_foraging_controller::social_foraging_controller(void) :
    m_rng(),
    m_fsm(),
    m_params(),
    m_actuators(GetActuator<CCI_DifferentialSteeringActuator>("differential_steering"),
                GetActuator<CCI_LEDsActuator>("leds"),
                GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing")),
    m_sensors(GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
              GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
              GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
              GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")),
),
    m_food_stats() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

NS_END(fordyca);
