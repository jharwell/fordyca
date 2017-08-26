/**
 * @file base_controller.cpp
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
#include "fordyca/controller/base_controller.hpp"
#include "fordyca/params/base_controller_repository.hpp"
#include "fordyca/representation/line_of_sight.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_controller::drop_block_in_nest(void) {
  ER_NOM("%s dropped block in nest", GetId().c_str());
  m_block = nullptr;
} /* drop_block_in_nest() */

void base_controller::pickup_block(representation::block* block) {
  ER_NOM("%s picked up block%d", GetId().c_str(), block->id());
  m_block = block;
} /* pickup_block() */

void base_controller::publish_event(enum event_type type) {
  switch (type) {
    case CONTINUE:
      m_fsm->event_continue();
      break;
    case BLOCK_FOUND:
      m_fsm->event_block_found();
      break;
  }
} /* publish_event() */

void base_controller::Init(argos::TConfigurationNode& node) {
  deferred_init(m_server);
  ER_NOM("Initializing foraging controller");

  params::base_controller_repository param_repo;
  param_repo.parse_all(node);
  param_repo.show_all(server_handle()->log_stream());

  m_actuators.reset(new actuator_manager(
      static_cast<const struct actuator_params*>(
          param_repo.get_params("actuators")),
      GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering"),
      GetActuator<argos::CCI_LEDsActuator>("leds"),
      GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing")));
  m_sensors.reset(new sensor_manager(
      static_cast<const struct sensor_params*>(
          param_repo.get_params("sensors")),
      GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
      GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
      GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
      GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")));

  m_fsm.reset(
      new foraging_fsm(static_cast<const struct foraging_fsm_params*>(
          param_repo.get_params("fsm")),
                     m_server,
                     m_sensors,
                     m_actuators));
  Reset();
} /* Init() */

void base_controller::Reset(void) {
  insmod("controller");
  server_handle()->mod_dbglvl(er_id(), rcppsw::common::er_lvl::DIAG);
  server_handle()->mod_loglvl(er_id(), rcppsw::common::er_lvl::VER);
  server_handle()->change_logfile(std::string(std::string("controller-") +
                                              GetId() +
                                              std::string(".txt")));
  m_fsm->init();
  m_block = nullptr;
} /* Reset() */

/* Notifiy ARGoS of the existence of the controller. */
using namespace argos;
REGISTER_CONTROLLER(base_controller, "base_controller");

NS_END(controller, fordyca);
