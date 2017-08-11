/**
 * @file foraging_controller.cpp
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
#include "fordyca/controller/foraging_controller.hpp"
#include "fordyca/params/actuator_param_parser.hpp"
#include "fordyca/params/sensor_param_parser.hpp"
#include "fordyca/params/fsm_param_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
foraging_controller::foraging_controller(void) :
    er_client(),
    m_param_manager(),
    m_actuators(),
    m_sensors(),
    m_fsm(),
    m_server(new rcppsw::common::er_server("init.txt")),
    m_block_data() {
  deferred_init(m_server);
  m_param_manager.logging_init(m_server);
  m_param_manager.add_category("actuators",
                               new params::actuator_param_parser());
  m_param_manager.add_category("sensors",
                               new params::sensor_param_parser());
  m_param_manager.add_category("fsm",
                               new params::fsm_param_parser());
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_controller::drop_block_in_nest(void) {
  ER_NOM("%s dropped block in nest", GetId().c_str());
  m_block_data.dropped_in_nest();
} /* drop_block_in_nest() */

void foraging_controller::pickup_block(int i) {
  ER_NOM("%s picked up block", GetId().c_str());
  m_block_data.picked_up_block(i);
} /* pickup_block() */

void foraging_controller::publish_event(enum event_type type) {
  switch (type) {
    case EXPLORE:
      m_fsm->event_explore();
      break;
    case CONTINUE:
      m_fsm->event_continue();
      break;
    case BLOCK_FOUND:
      m_fsm->event_block_found();
      break;
  }
} /* publish_event() */

void foraging_controller::Init(argos::TConfigurationNode& node) {
  ER_NOM("Initializing foraging controller");

  m_param_manager.parse_all(node);
  m_param_manager.show_all();
  m_actuators.reset(new actuator_manager(
      static_cast<const struct actuator_params*>(m_param_manager.get_params("actuators")),
      GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering"),
      GetActuator<argos::CCI_LEDsActuator>("leds"),
      GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing")));
  m_sensors.reset(new sensor_manager(
      static_cast<const struct sensor_params*>(m_param_manager.get_params("sensors")),
      GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
      GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
      GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
      GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")));

  m_fsm.reset(
      new foraging_fsm(static_cast<const struct foraging_fsm_params*>(m_param_manager.get_params("fsm")),
                     m_server,
                     m_sensors,
                     m_actuators));
  Reset();
} /* Init() */

void foraging_controller::Reset(void) {
  insmod("controller");
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::DIAG);
  server_handle()->mod_loglvl(er_id(), rcppsw::common::er_lvl::VER);
  server_handle()->change_logfile(std::string(std::string("controller-") +
                                              GetId() +
                                              std::string(".txt")));
  m_fsm->init();
  m_block_data.reset();
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
REGISTER_CONTROLLER(foraging_controller, "foraging_controller")

NS_END(controller, fordyca);
