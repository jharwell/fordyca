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
#include "fordyca/actuator_param_parser.hpp"
#include "fordyca/sensor_param_parser.hpp"
#include "fordyca/fsm_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
social_foraging_controller::social_foraging_controller(void) :
    m_rng(),
    m_parser(),
    m_actuators(),
    m_sensors(),
    m_fsm(),
    m_food_stats() {
  m_parser.add_category("actuators", actuator_param_parser());
  m_parser.add_category("sensors", sensor_param_parser());
  m_parser.add_category("fsm", fsm_param_parser());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void social_foraging_controller::Init(argos::TConfigurationNode& node) {
  m_parser.parse_all(node);
  m_actuators.reset(new actuator_manager(GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering"),
                                      GetActuator<argos::CCI_LEDsActuator>("leds"),
                                      GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing"),
                                         static_cast<const struct actuator_params&>(*m_parser.get_params("actuators"))));

  m_sensors.reset(new sensor_manager(GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
                                     GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
                                     GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
                                     GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground"),
                                     static_cast<const struct sensor_params&>(*m_parser.get_params("sensors"))));
  m_fsm.reset(new social_fsm(static_cast<const struct social_fsm_params&>(*m_parser.get_params("fsm")),
                             m_sensors.get(),
                             m_actuators.get()));
  Reset();
} /* Init() */

void social_foraging_controller::Reset(void) {
  m_fsm->reset();
  m_food_stats.reset();
  m_actuators->leds_set_color(argos::CColor::WHITE);
  m_fsm->event_explore();
} /* Reset() */


NS_END(fordyca);
