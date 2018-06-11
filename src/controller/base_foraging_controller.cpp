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
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <experimental/filesystem>
#include <fstream>

#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/actuation_params.hpp"
#include "fordyca/params/depth0/stateless_foraging_repository.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fs = std::experimental::filesystem;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_foraging_controller::base_foraging_controller(void)
    : m_server(std::make_shared<rcppsw::er::server>()) {
  client::deferred_client_init(m_server);

  /* diagnostic for logging, nominal for printing */
  client::insmod("controller", rcppsw::er::er_lvl::DIAG, rcppsw::er::er_lvl::NOM);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_foraging_controller::in_nest(void) const {
  return m_saa->sensing()->in_nest();
} /* in_nest() */

bool base_foraging_controller::block_detected(void) const {
  return m_saa->sensing()->block_detected();
} /* block_detected() */

void base_foraging_controller::robot_loc(argos::CVector2 loc) {
  m_saa->sensing()->position(loc);
}

__pure argos::CVector2 base_foraging_controller::robot_loc(void) const {
  return m_saa->sensing()->position();
}

void base_foraging_controller::Init(ticpp::Element& node) {
  ER_NOM("Initializing base foraging controller");
  params::depth0::stateless_foraging_repository param_repo;
  param_repo.parse_all(node);
  ER_ASSERT(param_repo.validate_all(),
            "FATAL: Not all parameters were validated");

  /* initialize output */
  const struct params::output_params* params =
      param_repo.parse_results<struct params::output_params>();
  client::server_handle()->log_stream() << param_repo;
  output_init(params);

  /* initialize sensing and actuation subsystem */
  struct actuation_subsystem::actuator_list alist = {
    .wheels = GetActuator<argos::CCI_DifferentialSteeringActuator>(
        "differential_steering"),
    .leds = GetActuator<argos::CCI_LEDsActuator>("leds"),
    .raba = GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing")
  };
  struct base_sensing_subsystem::sensor_list slist = {
    .rabs = GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing"),
    .proximity = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity"),
    .light = GetSensor<argos::CCI_FootBotLightSensor>("footbot_light"),
    .ground = GetSensor<argos::CCI_FootBotMotorGroundSensor>("footbot_motor_ground")
  };
  m_saa = std::make_shared<controller::saa_subsystem>(
      m_server,
      param_repo.parse_results<struct params::actuation_params>(),
      param_repo.parse_results<struct params::sensing_params>(),
      &alist,
      &slist);

  this->Reset();
  ER_NOM("Base foraging controller initialization finished");
} /* Init() */

void base_foraging_controller::Reset(void) {
  CCI_Controller::Reset();
  m_block.reset();
} /* Reset() */

void base_foraging_controller::output_init(
    const struct params::output_params* const params) {
  std::string output_root;
  if ("__current_date__" == params->output_dir) {
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    output_root = params->output_root + "/" + std::to_string(now.date().year()) +
                  "-" + std::to_string(now.date().month()) + "-" +
                  std::to_string(now.date().day()) + ":" +
                  std::to_string(now.time_of_day().hours()) + "-" +
                  std::to_string(now.time_of_day().minutes());
  } else {
    output_root = params->output_root + "/" + params->output_dir;
  }

  if (!fs::exists(output_root)) {
    fs::create_directories(output_root);
  }

  /* setup logging timestamp calculator */
  client::server_handle()->log_ts_calculator(
      std::bind(&base_foraging_controller::log_header_calc, this));
  client::server_handle()->dbg_ts_calculator(
      std::bind(&base_foraging_controller::dbg_header_calc, this));

  client::server_handle()->change_logfile(output_root + "/" + this->GetId() +
                                          ".log");
} /* output_init() */

std::string base_foraging_controller::log_header_calc(void) const {
  return "[t=" + std::to_string(m_saa->sensing()->tick()) + "," + this->GetId() + "]";
} /* log_header_calc() */

std::string base_foraging_controller::dbg_header_calc(void) const {
  return this->GetId();
} /* dbg_header_calc() */

void base_foraging_controller::tick(uint tick) {
  m_saa->sensing()->tick(tick);
} /* tick() */

NS_END(controller, fordyca);
