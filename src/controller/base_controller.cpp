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
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <experimental/filesystem>
#include <fstream>

#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/actuation_params.hpp"
#include "fordyca/params/base_controller_repository.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fs = std::experimental::filesystem;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller::base_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.base"), m_saa(nullptr) {}

base_controller::~base_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_controller::in_nest(void) const {
  return m_saa->sensing()->in_nest();
} /* in_nest() */

bool base_controller::block_detected(void) const {
  return m_saa->sensing()->block_detected();
} /* block_detected() */

void base_controller::position(const rmath::vector2d& loc) {
  m_saa->sensing()->position(loc);
}
void base_controller::discrete_position(const rmath::vector2u& loc) {
  m_saa->sensing()->discrete_position(loc);
}

__rcsw_pure const rmath::vector2d& base_controller::position(void) const {
  return m_saa->sensing()->position();
}
__rcsw_pure const rmath::vector2u& base_controller::discrete_position(void) const {
  return m_saa->sensing()->discrete_position();
}

__rcsw_pure rmath::vector2d base_controller::heading(void) const {
  return m_saa->sensing()->heading();
}

void base_controller::Init(ticpp::Element& node) {
#ifndef RCPPSW_ER_NREPORT
  if (const char* env_p = std::getenv("LOG4CXX_CONFIGURATION")) {
    client<std::remove_reference<decltype(*this)>::type>::init_logging(env_p);
  } else {
    std::cerr << "LOG4CXX_CONFIGURATION not defined" << std::endl;
    std::exit(EXIT_FAILURE);
  }
#endif

  params::base_controller_repository param_repo;
  param_repo.parse_all(node);

  ndc_push();
  if (!param_repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize output */
  auto* params = param_repo.parse_results<struct params::output_params>();
  output_init(params);

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(param_repo.parse_results<params::actuation_params>(),
           param_repo.parse_results<params::sensing_params>());
  ndc_pop();
} /* Init() */

void base_controller::Reset(void) {
  CCI_Controller::Reset();
  m_block.reset();
} /* Reset() */

void base_controller::saa_init(const params::actuation_params* const actuation_p,
                               const params::sensing_params* const sensing_p) {
  struct actuation_subsystem::actuator_list alist = {
      .wheels = hal::actuators::differential_drive_actuator(
          GetActuator<argos::CCI_DifferentialSteeringActuator>(
              "differential_steering")),
#ifdef FORDYCA_WITH_ROBOT_LEDS
      .leds = hal::actuators::led_actuator(
          GetActuator<argos::CCI_LEDsActuator>("leds")),
#else
      .leds = hal::actuators::led_actuator(nullptr),
#endif /* FORDYCA_WITH_ROBOT_LEDS */

#ifdef FORDYCA_WITH_ROBOT_RAB
      .wifi = hal::actuators::wifi_actuator(
          GetActuator<argos::CCI_RangeAndBearingActuator>("range_and_bearing")),
#else
      .wifi = hal::actuators::wifi_actuator(nullptr)
#endif /* FORDYCA_WITH_ROBOT_RAB */
  };
  struct sensing_subsystem::sensor_list slist = {
#ifdef FORDYCA_WITH_ROBOT_RAB
      .rabs = hal::sensors::rab_wifi_sensor(
          GetSensor<argos::CCI_RangeAndBearingSensor>("range_and_bearing")),
#else
      .rabs = hal::sensors::rab_wifi_sensor(nullptr),
#endif /* FORDYCA_WITH_ROBOT_RAB */
      .proximity = hal::sensors::proximity_sensor(
          GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity")),
      .light = hal::sensors::light_sensor(
          GetSensor<argos::CCI_FootBotLightSensor>("footbot_light")),
      .ground = hal::sensors::ground_sensor(
          GetSensor<argos::CCI_FootBotMotorGroundSensor>(
              "footbot_motor_ground")),
#ifdef FORDYCA_WITH_ROBOT_BATTERY
      .battery = hal::sensors::battery_sensor(
          GetSensor<argos::CCI_BatterySensor>("battery")),
#else
      .battery = hal::sensors::battery_sensor(nullptr),
#endif /* FORDYCA_WITH_ROBOT_BATTERY */
  };
  m_saa = rcppsw::make_unique<controller::saa_subsystem>(
      actuation_p, sensing_p, &alist, &slist);
} /* saa_init() */

void base_controller::output_init(
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

#ifndef RCPPSW_ER_NREPORT
  /*
   * Each file appender is attached to a root category in the FORDYCA
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  client::set_logfile(log4cxx::Logger::getLogger("fordyca.controller"),
                      output_root + "/controller.log");
  client::set_logfile(log4cxx::Logger::getLogger("fordyca.ds"),
                      output_root + "/ds.log");

  client::set_logfile(log4cxx::Logger::getLogger("rcppsw.ta"),
                      output_root + "/ta.log");

  client::set_logfile(log4cxx::Logger::getLogger("fordyca.fsm"),
                      output_root + "/fsm.log");

  client::set_logfile(log4cxx::Logger::getLogger(
                          "fordyca.controller.saa_subsystem"),
                      output_root + "/saa.log");
  client::set_logfile(log4cxx::Logger::getLogger(
                          "fordyca.controller.explore_behavior"),
                      output_root + "/saa.log");
#endif
} /* output_init() */

void base_controller::tick(uint tick) {
  m_saa->sensing()->tick(tick);
} /* tick() */

int base_controller::entity_id(void) const {
  return std::atoi(GetId().c_str() + 2);
} /* entity_id() */

void base_controller::ndc_pusht(void) {
  ER_NDC_PUSH("[t=" + std::to_string(m_saa->sensing()->tick()) + "] [" +
              GetId() + "]");
}

double base_controller::applied_motion_throttle(void) const {
  return saa_subsystem()->actuation()->differential_drive().applied_throttle();
} /* applied_motion_throttle() */

void base_controller::tv_init(const support::tv::tv_manager* tv_manager) {
  m_tv_manager = tv_manager;
  saa_subsystem()->actuation()->differential_drive().throttling(
      tv_manager->movement_throttling_handler(entity_id()));
} /* tv_init() */

/*******************************************************************************
 * Movement Metrics
 ******************************************************************************/
__rcsw_pure double base_controller::distance(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa_subsystem()->sensing()->tick() > 1) {
    return saa_subsystem()->sensing()->heading().length();
  }
  return 0;
} /* distance() */

rmath::vector2d base_controller::velocity(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa_subsystem()->sensing()->tick() > 1) {
    return saa_subsystem()->linear_velocity();
  }
  return {0, 0};
} /* velocity() */

NS_END(controller, fordyca);
