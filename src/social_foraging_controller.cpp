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
    er_client(),
    m_parser(),
    m_actuators(),
    m_sensors(),
    m_fsm(),
    m_server(new rcppsw::common::er_server(
        std::string(std::string("controller-") +
                    this->GetId() +
                    std::string(".txt")) ,
        rcppsw::common::er_lvl::NOM,
        rcppsw::common::er_lvl::NOM, false)),
    m_food_stats() {
  deferred_init(m_server);
  m_parser.add_category("actuators", new actuator_param_parser());
  m_parser.add_category("sensors", new sensor_param_parser());
  m_parser.add_category("fsm", new fsm_param_parser());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void social_foraging_controller::Init(argos::TConfigurationNode& node) {
  ER_NOM("Initializing social foraging controller\n");

  m_parser.parse_all(node);
  ER_NOM(" - Parsed all parameters\n");

  m_actuators.reset(new actuator_manager(
      static_cast<const struct actuator_params*>(m_parser.get_params("actuators")),
      GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering"),
      GetActuator<argos::CCI_LEDsActuator>("leds"),
      GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing")));

  m_sensors.reset(new sensor_manager(
      static_cast<const struct sensor_params*>(m_parser.get_params("sensors")),
      GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
      GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
      GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
      GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")));

  m_fsm.reset(
      new social_fsm(static_cast<const struct social_fsm_params*>(m_parser.get_params("fsm")),
                     m_server,
                     m_sensors.get(),
                     m_actuators.get()));
  ER_NOM(" - Loaded all sensors and actuators\n");
  Reset();
} /* Init() */

void social_foraging_controller::Reset(void) {
  m_fsm->reset();
  m_food_stats.reset();
  m_actuators->leds_set_color(argos::CColor::WHITE);
  ER_NOM("Reset social foraging controller\n");
} /* Reset() */

/*
 * This statement notifies ARGoS of the existence of the controller.  It binds
 * the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this
 * controller.  When ARGoS reads that string in the XML file, it knows which
 * controller class to instantiate.  See also the XML configuration files for an
 * example of how this is used.
 */
using namespace argos;
REGISTER_CONTROLLER(social_foraging_controller, "social_foraging_controller")

NS_END(fordyca);
