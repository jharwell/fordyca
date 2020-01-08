/**
 * \file base_controller.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <experimental/filesystem>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/metrics/config/output_config.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/subsystem/config/sensing_subsystem2D_config.hpp"
#include "cosm/subsystem/saa_subsystem2D.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"

#include "fordyca/config/base_controller_repository.hpp"
#include "fordyca/config/saa_xml_names.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fs = std::experimental::filesystem;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller::base_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.base"), m_block(nullptr) {}

base_controller::~base_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_controller::in_nest(void) const {
  return saa()->sensing()->ground()->detect(
      chal::sensors::ground_sensor::kNestTarget);
} /* in_nest() */

bool base_controller::block_detected(void) const {
  return saa()->sensing()->ground()->detect("block");
} /* block_detected() */

void base_controller::init(ticpp::Element& node) {
#if (LIBRA_ER == LIBRA_ER_ALL)
  if (const char* env_p = std::getenv("LOG4CXX_CONFIGURATION")) {
    client<std::remove_reference<decltype(*this)>::type>::init_logging(env_p);
  } else {
    std::cerr << "LOG4CXX_CONFIGURATION not defined" << std::endl;
    std::exit(EXIT_FAILURE);
  }
#endif

  config::base_controller_repository repo;
  repo.parse_all(node);

  ndc_push();
  if (!repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize RNG */
  auto rngp = repo.config_get<rmath::config::rng_config>();
  base_controller2D::rng_init((nullptr == rngp) ? -1 : rngp->seed, "footbot");

  /* initialize output */
  auto* outputp = repo.config_get<cmconfig::output_config>();
  base_controller2D::output_init(outputp->output_root, outputp->output_dir);

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(repo.config_get<csubsystem::config::actuation_subsystem2D_config>(),
           repo.config_get<csubsystem::config::sensing_subsystem2D_config>());
  ndc_pop();
} /* init() */

void base_controller::reset(void) { m_block.reset(); } /* Reset() */

void base_controller::output_init(const cmconfig::output_config* outputp) {
  std::string dir =
      base_controller2D::output_init(outputp->output_root, outputp->output_dir);

#if (LIBRA_ER == LIBRA_ER_ALL)
  /*
   * Each file appender is attached to a root category in the COSM
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  client<base_controller>::set_logfile(log4cxx::Logger::getLogger("rcppsw.ta"),
                                       dir + "/ta.log");

  client<base_controller>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.controller"), dir + "/controller.log");
  client<base_controller>::set_logfile(log4cxx::Logger::getLogger("fordyca.ds"),
                                       dir + "/ds.log");
  client<base_controller>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.fsm"), dir + "/fsm.log");
  client<base_controller>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.controller.saa"), dir + "/saa.log");
  client<base_controller>::set_logfile(
      log4cxx::Logger::getLogger("fordyca.controller.explore_behavior"),
      dir + "/saa.log");
#endif
} /* output_init() */

void base_controller::saa_init(
    const csubsystem::config::actuation_subsystem2D_config* actuation_p,
    const csubsystem::config::sensing_subsystem2D_config* sensing_p) {
  auto saa_names = config::saa_xml_names();
#ifdef FORDYCA_WITH_ROBOT_RAB
  /* auto rabs = chal::sensors::wifi_sensor( */
  /*     GetSensor<argos::CCI_RangeAndBearingSensor>(saa_names.rab_saa)); */
  auto rabs = nullptr;
#else
  auto rabs = chal::sensors::wifi_sensor(nullptr);
#endif /* FORDYCA_WITH_ROBOT_RAB */

#ifdef FORDYCA_WITH_ROBOT_BATTERY
  auto battery = chal::sensors::battery_sensor(
      GetSensor<argos::CCI_BatterySensor>(saa_names.battery_sensor));
#else
  auto battery = chal::sensors::battery_sensor(nullptr);
#endif /* FORDYCA_WITH_ROBOT_BATTERY */

  auto proximity = chal::sensors::proximity_sensor(
      GetSensor<argos::CCI_FootBotProximitySensor>(saa_names.prox_sensor),
      &sensing_p->proximity);
  auto blobs = chal::sensors::colored_blob_camera_sensor(
      GetSensor<argos::CCI_ColoredBlobOmnidirectionalCameraSensor>(
          saa_names.camera_sensor));
  auto light = chal::sensors::light_sensor(
      GetSensor<argos::CCI_FootBotLightSensor>(saa_names.light_sensor));
  auto ground = chal::sensors::ground_sensor(
      GetSensor<argos::CCI_FootBotMotorGroundSensor>(saa_names.ground_sensor),
      &sensing_p->ground);

  auto diff_drives = chal::sensors::diff_drive_sensor(
      GetSensor<argos::CCI_DifferentialSteeringSensor>(
          saa_names.diff_steering_saa));

  auto sensors = csubsystem::sensing_subsystem2D::sensor_map{
      csubsystem::sensing_subsystem2D::map_entry_create(rabs),
      csubsystem::sensing_subsystem2D::map_entry_create(battery),
      csubsystem::sensing_subsystem2D::map_entry_create(proximity),
      csubsystem::sensing_subsystem2D::map_entry_create(blobs),
      csubsystem::sensing_subsystem2D::map_entry_create(light),
      csubsystem::sensing_subsystem2D::map_entry_create(ground),
      csubsystem::sensing_subsystem2D::map_entry_create(diff_drives)};

  auto diff_drivea = ckin2D::governed_diff_drive(
      &actuation_p->diff_drive,
      chal::actuators::diff_drive_actuator(
          GetActuator<argos::CCI_DifferentialSteeringActuator>(
              saa_names.diff_steering_saa)),
      ckin2D::governed_diff_drive::drive_type::kFSMDrive);

#ifdef FORDYCA_WITH_ROBOT_LEDS
  auto leds = chal::actuators::led_actuator(
      GetActuator<argos::CCI_LEDsActuator>(saa_names.leds_saa));
#else
  auto leds = chal::actuators::led_actuator(nullptr);
#endif /* FORDYCA_WITH_ROBOT_LEDS */

#ifdef FORDYCA_WITH_ROBOT_RAB
  auto raba = chal::actuators::wifi_actuator(
      GetActuator<argos::CCI_RangeAndBearingActuator>(saa_names.rab_saa));

#else
  auto raba = chal::actuators::wifi_actuator(nullptr);
#endif /* FORDYCA_WITH_ROBOT_RABS */

  auto actuators = csubsystem::actuation_subsystem2D::actuator_map{
      /*
     * We put the governed differential drive in the actuator map twice because
     * some of the reusable components use the base class differential drive
     * instead of the governed version (no robust way to inform that we want to
     * use the governed version).
     */
      csubsystem::actuation_subsystem2D::map_entry_create(diff_drivea),
      /* {typeid(chal::actuators::diff_drive_actuator), */
      /*  csubsystem::actuation_subsystem2D::variant_type(diff_drivea)}, */

      csubsystem::actuation_subsystem2D::map_entry_create(leds),
      csubsystem::actuation_subsystem2D::map_entry_create(raba)};

  base_controller2D::saa(std::make_unique<crfootbot::footbot_saa_subsystem>(
      sensors, actuators, &actuation_p->steering));
} /* saa_init() */

rtypes::type_uuid base_controller::entity_id(void) const {
  return rtypes::type_uuid(std::atoi(GetId().c_str() + 2));
} /* entity_id() */

double base_controller::applied_movement_throttle(void) const {
  return saa()->actuation()->governed_diff_drive()->applied_throttle();
} /* applied_movement_throttle() */

void base_controller::irv_init(const ctv::robot_dynamics_applicator* rda) {
  if (rda->motion_throttling_enabled()) {
    saa()->actuation()->governed_diff_drive()->tv_generator(
        rda->motion_throttler(entity_id()));
  }
} /* irv_init() */

void base_controller::block(std::unique_ptr<crepr::base_block2D> block) {
  m_block = std::move(block);
}

std::unique_ptr<crepr::base_block2D> base_controller::block_release(void) {
  return std::move(m_block);
}

class crfootbot::footbot_saa_subsystem* base_controller::saa(void) {
  return static_cast<crfootbot::footbot_saa_subsystem*>(
      base_controller2D::saa());
}
const class crfootbot::footbot_saa_subsystem* base_controller::saa(void) const {
  return static_cast<const crfootbot::footbot_saa_subsystem*>(
      base_controller2D::saa());
}
NS_END(controller, fordyca);
