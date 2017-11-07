/**
 * @file base_foraging_controller.cpp
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
#include "fordyca/controller/base_foraging_controller.hpp"
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>

#include "fordyca/controller/base_foraging_sensor_manager.hpp"
#include "fordyca/controller/actuator_manager.hpp"
#include "fordyca/params/base_foraging_repository.hpp"
#include "fordyca/params/actuator_params.hpp"
#include "fordyca/params/sensor_params.hpp"
#include "rcppsw/common/er_server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_foraging_controller::base_foraging_controller(void) :
    er_client(),
    m_display_los(false),
    m_display_id(false),
    m_block(nullptr),
    m_actuators(),
    m_sensors(),
    m_server(std::make_shared<rcppsw::common::er_server>()) {
  /*
   * Initially, all robots use the RCPPSW er_server to log parameters.
   */
  er_client::deferred_init(m_server);

  /* diagnostic for logging, nominal for printing */
  er_client::insmod("controller",
         rcppsw::common::er_lvl::DIAG,
         rcppsw::common::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_foraging_controller::in_nest(void) const {
  return m_sensors->in_nest();
} /* in_nest() */

bool base_foraging_controller::block_detected(void) const {
  return m_sensors->block_detected();
} /* block_detected() */

void base_foraging_controller::Init(argos::TConfigurationNode& node) {
  er_client::server_handle()->change_logfile(std::string(std::string("controller-") +
                                                         GetId() +
                                              std::string(".txt")));
  ER_NOM("Initializing base foraging controller");
  params::base_foraging_repository param_repo;
  param_repo.parse_all(node);
  param_repo.show_all(er_client::server_handle()->log_stream());


  m_actuators.reset(new actuator_manager(
      static_cast<const struct params::actuator_params*>(
          param_repo.get_params("actuators")),
      GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering"),
      GetActuator<argos::CCI_LEDsActuator>("leds"),
      GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing")));
  m_sensors.reset(new base_foraging_sensor_manager(
      static_cast<const struct params::sensor_params*>(
          param_repo.get_params("sensors")),
      GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
      GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
      GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
      GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")));

  CCI_Controller::Reset();
  ER_NOM("Base foraging controller initialization finished");
} /* Init() */

void base_foraging_controller::Reset(void) {
  m_block = nullptr;
} /* Reset() */

NS_END(controller, fordyca);
