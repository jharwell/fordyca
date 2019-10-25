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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <experimental/filesystem>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "fordyca/config/base_controller_repository.hpp"
#include "fordyca/config/output_config.hpp"
#include "fordyca/config/saa_xml_names.hpp"
#include "fordyca/repr/base_block.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/subsystem/config/sensing_subsystem2D_config.hpp"
#include "cosm/subsystem/saa_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace fs = std::experimental::filesystem;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller::base_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.base"),
      m_block(nullptr),
      m_saa(nullptr) {}

base_controller::~base_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool base_controller::in_nest(void) const {
  return m_saa->sensing()->ground()->detect(
      chal::sensors::ground_sensor::kNestTarget);
} /* in_nest() */

bool base_controller::block_detected(void) const {
  return m_saa->sensing()->ground()->detect("block");
} /* block_detected() */

void base_controller::position(const rmath::vector2d& loc) {
  m_saa->sensing()->position(loc);
}
void base_controller::heading(const rmath::radians& h) {
  m_saa->sensing()->heading(h);
}
void base_controller::discrete_position(const rmath::vector2u& loc) {
  m_saa->sensing()->discrete_position(loc);
}

void base_controller::Init(ticpp::Element& node) {
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
  rng_init(repo.config_get<rmath::config::rng_config>());

  /* initialize output */
  auto* config = repo.config_get<config::output_config>();
  output_init(config);

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(repo.config_get<csubsystem::config::actuation_subsystem2D_config>(),
           repo.config_get<csubsystem::config::sensing_subsystem2D_config>());
  ndc_pop();
} /* Init() */

void base_controller::Reset(void) {
  CCI_Controller::Reset();
  m_block.reset();
} /* Reset() */

void base_controller::saa_init(
    const csubsystem::config::actuation_subsystem2D_config* actuation_p,
    const csubsystem::config::sensing_subsystem2D_config* sensing_p) {
  auto saa_names = config::saa_xml_names();
#ifdef FORDYCA_WITH_ROBOT_RAB
  auto rabs = chal::sensors::wifi_sensor(
      GetSensor<argos::CCI_RangeAndBearingSensor>(saa_names.rab_saa));
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
     * instead of the governed entry (no robust way to inform that we want to
     * use the governed version).n
     */
      csubsystem::actuation_subsystem2D::map_entry_create(diff_drivea),
      {typeid(chal::actuators::diff_drive_actuator),
       csubsystem::actuation_subsystem2D::variant_type(diff_drivea)},

      csubsystem::actuation_subsystem2D::map_entry_create(leds),
      csubsystem::actuation_subsystem2D::map_entry_create(raba)};

  m_saa = std::make_unique<crfootbot::footbot_saa_subsystem>(
      sensors, actuators, &actuation_p->steering);
} /* saa_init() */

void base_controller::output_init(const config::output_config* const config) {
  std::string output_root;
  if ("__current_date__" == config->output_dir) {
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    output_root = config->output_root + "/" + std::to_string(now.date().year()) +
                  "-" + std::to_string(now.date().month()) + "-" +
                  std::to_string(now.date().day()) + ":" +
                  std::to_string(now.time_of_day().hours()) + "-" +
                  std::to_string(now.time_of_day().minutes());
  } else {
    output_root = config->output_root + "/" + config->output_dir;
  }

  if (!fs::exists(output_root)) {
    fs::create_directories(output_root);
  }

#if (LIBRA_ER == LIBRA_ER_ALL)
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

void base_controller::rng_init(const rmath::config::rng_config* config) {
  rmath::rngm::instance().register_type<rmath::rng>("footbot");
  if (nullptr == config || (nullptr != config && -1 == config->seed)) {
    ER_INFO("Using time seeded RNG");
    m_rng = rmath::rngm::instance().create(
        "footbot", std::chrono::system_clock::now().time_since_epoch().count());
  } else {
    /*
     * We add the entity ID to the configured seed to ensure that all robots
     * have unique random number sequences they are drawing from (otherwise they
     * can all chose to allocate the same initial task, for example), while
     * still maintaining reproducibility.
     */
    ER_INFO("Using user seeded RNG");
    m_rng = rmath::rngm::instance().create("footbot", config->seed);
  }
} /* rng_init() */

void base_controller::tick(rtypes::timestep tick) {
  m_saa->sensing()->tick(tick);
} /* tick() */

int base_controller::entity_id(void) const {
  return std::atoi(GetId().c_str() + 2);
} /* entity_id() */

void base_controller::ndc_pusht(void) {
  ER_NDC_PUSH(std::string("[t=") + std::to_string(m_saa->sensing()->tick().v()) +
              std::string("] [") + GetId() + std::string("]"));
}

double base_controller::applied_movement_throttle(void) const {
  return saa()->actuation()->governed_diff_drive()->applied_throttle();
} /* applied_movement_throttle() */

void base_controller::tv_init(const support::tv::tv_manager* tv_manager) {
  if (tv_manager->movement_throttling_enabled()) {
    saa()->actuation()->governed_diff_drive()->tv_generator(
        tv_manager->movement_throttling_handler(entity_id()));
  }
} /* tv_init() */

void base_controller::block(std::unique_ptr<repr::base_block> block) {
  m_block = std::move(block);
}

std::unique_ptr<repr::base_block> base_controller::block_release(void) {
  return std::move(m_block);
}
/*******************************************************************************
 * Movement Metrics
 ******************************************************************************/
rtypes::spatial_dist base_controller::distance(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa()->sensing()->tick() > 1) {
    return rtypes::spatial_dist(saa()->sensing()->tick_travel().length());
  }
  return rtypes::spatial_dist(0.0);
} /* distance() */

rmath::vector2d base_controller::velocity(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa()->sensing()->tick() > 1) {
    return saa()->linear_velocity();
  }
  return {0, 0};
} /* velocity() */

/*******************************************************************************
 * Swarm Spatial Metrics
 ******************************************************************************/
const rmath::vector2d& base_controller::position2D(void) const {
  return m_saa->sensing()->position();
}
const rmath::vector2u& base_controller::discrete_position2D(void) const {
  return m_saa->sensing()->discrete_position();
}

rmath::vector2d base_controller::heading2D(void) const {
  return {1.0, m_saa->sensing()->heading()};
}

NS_END(controller, fordyca);
