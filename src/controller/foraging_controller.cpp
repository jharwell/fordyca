/**
 * \file foraging_controller.cpp
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
#include "fordyca//controller/foraging_controller.hpp"

#include <filesystem>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/hal/argos/subsystem/config/xml/saa_names.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/hal/subsystem/config/sensing_subsystemQ3D_config.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"
#include "cosm/spatial/interference_tracker.hpp"
#include "cosm/spatial/nest_zone_tracker.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"

#include "fordyca/controller/config/foraging_controller_repository.hpp"
#include "fordyca/repr/diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Static Functions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
csubsystem::sensing_subsystemQ3D::sensor_map
static sensing_init(foraging_controller* c,
             const chal::subsystem::config::sensing_subsystemQ3D_config* sensing) {
  using saa_names = chargos::subsystem::config::xml::saa_names;
  auto proximity_handle = c->GetSensor<chargos::sensors::ir_sensor::impl_type>(
      saa_names::ir_sensor);
  auto light_handle = c->GetSensor<chargos::sensors::light_sensor::impl_type>(
      saa_names::light_sensor);
  auto env_handle = c->GetSensor<chal::sensors::env_sensor::impl_type>(
      saa_names:: ground_sensor);
  auto position_handle = c->GetSensor<chargos::sensors::position_sensor::impl_type>(
      saa_names::position_sensor);
  auto steering_handle = c->GetSensor<chargos::sensors::diff_drive_sensor::impl_type>(
      saa_names::diff_steering_saa);

#if defined(FORDYCA_WITH_ROBOT_RAB)
  auto rab_handle = c->GetSensor<chsensors::wifi_sensor::impl_type>(
      saa_names::rab_saa);
#else
  auto rab_handle = nullptr;
#endif /* FORDYCA_WITH_ROBOT_RAB */

#if defined(FORDYCA_WITH_ROBOT_BATTERY)
  auto battery_handle = c->GetSensor<chsensors::battery_sensor::impl_type>(
      saa_names::battery_sensor);
#else
  auto battery_handle = nullptr;
#endif /* FORDYCA_WITH_ROBOT_BATTERY */

#ifdef FORDYCA_WITH_ROBOT_CAMERA
  auto blobs_handle = c->GetSensor<chargos::sensors::colored_blob_camera_sensor::impl_type>(
      saa_names::camera_sensor);
#else
      auto blobs_handle = nullptr;
#endif

      auto proximity = chsensors::proximity_sensor(proximity_handle,
                                                   &sensing->proximity);
      auto light = chargos::sensors::light_sensor(light_handle);
      auto env = chal::sensors::env_sensor(env_handle, &sensing->env);
      auto position = chargos::sensors::position_sensor(position_handle);
      auto steering = chargos::sensors::diff_drive_sensor(steering_handle);
      auto odometry = chal::sensors::odometry_sensor(std::move(position),
                                                     std::move(steering));

      auto rabs = chargos::sensors::wifi_sensor(rab_handle);
      auto battery = chargos::sensors::battery_sensor(battery_handle);
      auto blobs = chargos::sensors::colored_blob_camera_sensor(blobs_handle);


      using subsystem = csubsystem::sensing_subsystemQ3D;
      subsystem::sensor_map sensors;
#if defined(FORDYCA_WITH_ROBOT_RAB)
      sensors.emplace(subsystem::map_entry_create(std::move(rabs)));
#endif

#if defined(FORDYCA_WITH_ROBOT_BATTERY)
      sensors.emplace(subsystem::map_entry_create(std::move(battery)));
#endif
      sensors.emplace(subsystem::map_entry_create(std::move(proximity)));
      sensors.emplace(subsystem::map_entry_create(std::move(blobs)));
      sensors.emplace(subsystem::map_entry_create(std::move(light)));
      sensors.emplace(subsystem::map_entry_create(std::move(env)));
      sensors.emplace(subsystem::map_entry_create(std::move(odometry)));

      return sensors;
} /* sensing_init() */
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

#if defined(COSM_HAL_TARGET_ROS_ROBOT)
csubsystem::sensing_subsystemQ3D::sensor_map
static sensing_init(foraging_controller* c,
             const chal::subsystem::config::sensing_subsystemQ3D_config* sensing) {
  auto robot_ns = cros::to_ns(c->entity_id());
  auto light = chros::sensors::light_sensor(robot_ns);
  auto sonar = chros::sensors::sonar_sensor(robot_ns);
  auto lidar = chros::sensors::lidar_sensor(robot_ns);
  auto proximity = chsensors::proximity_sensor(robot_ns,
                                               &sensing->proximity);
  auto odometry = chros::sensors::odometry_sensor(robot_ns);
  auto env = chsensors::env_sensor(robot_ns, &sensing->env);

  using subsystem = csubsystem::sensing_subsystemQ3D;
  subsystem::sensor_map sensors;
  sensors.emplace(subsystem::map_entry_create(std::move(light)));
  sensors.emplace(subsystem::map_entry_create(std::move(sonar)));
  sensors.emplace(subsystem::map_entry_create(std::move(lidar)));
  sensors.emplace(subsystem::map_entry_create(std::move(proximity)));
  sensors.emplace(subsystem::map_entry_create(std::move(odometry)));
  sensors.emplace(subsystem::map_entry_create(std::move(env)));
  return sensors;
} /* sensing_init() */
#endif /* COSM_HAL_TARGET_ROS_ROBOT */

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
csubsystem::actuation_subsystem2D::actuator_map
static actuation_init(foraging_controller* c,
                     const csubsystem::config::actuation_subsystem2D_config* actuation) {
  using saa_names = chargos::subsystem::config::xml::saa_names;

  auto diff_drive = ckin2D::governed_diff_drive(
      &actuation->diff_drive,
      chactuators::diff_drive_actuator(
          c->GetActuator<chactuators::diff_drive_actuator::impl_type>(
              saa_names::diff_steering_saa)));

#ifdef FORDYCA_WITH_ROBOT_LEDS
  auto diag_handle = c->GetActuator<chargos::actuators::led_actuator::impl_type>(saa_names::leds_saa);
#else
  auto diag_handle = nullptr;
#endif /* FORDYCA_WITH_ROBOT_LEDS */
  auto diag = chactuators::diagnostic_actuator(diag_handle,
                                               frepr::diagnostics::kColorMap);

#if defined(FORDYCA_WITH_ROBOT_RAB)
  auto rab_handle = c->GetActuator<chactuators::wifi_actuator::impl_type>(saa_names::rab_saa);
#else
  auto rab_handle = nullptr;
#endif /* FORDYCA_WITH_ROBOT_RAB */

  auto rab = chargos::actuators::wifi_actuator(rab_handle);

  using subsystem = csubsystem::actuation_subsystem2D;
  subsystem::actuator_map actuators;
  actuators.emplace(subsystem::map_entry_create(std::move(diff_drive)));
  actuators.emplace(subsystem::map_entry_create(std::move(diag)));

#if defined(FORDYCA_WITH_ROBOT_RAB)
  actuators.emplace(subsystem::map_entry_create(std::move(rab)));
#endif
  return actuators;
} /* actuation_init() */
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

#if defined(COSM_HAL_TARGET_ROS_ROBOT)
csubsystem::actuation_subsystem2D::actuator_map
static actuation_init(foraging_controller* c,
                     const csubsystem::config::actuation_subsystem2D_config* actuation) {
  auto diff_drive = ckin2D::governed_diff_drive(
      &actuation->diff_drive,
      chactuators::diff_drive_actuator(cros::to_ns(c->entity_id())));

#if defined(FORDYCA_WITH_ROBOT_LEDS)
  auto diag = chactuators::diagnostic_actuator(true);
#else
  auto diag = chactuators::diagnostic_actuator(false);
#endif

  using subsystem = csubsystem::actuation_subsystem2D;
  subsystem::actuator_map actuators;
  actuators.emplace(subsystem::map_entry_create(std::move(diff_drive)));
  actuators.emplace(subsystem::map_entry_create(std::move(diag)));
  return actuators;
} /* actuation_init() */
#endif /* COSM_HAL_TARGET_ROS_ROBOT */

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller::foraging_controller(void)
    : ER_CLIENT_INIT("fordyca.controller.foraging_controller") {}

foraging_controller::~foraging_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool foraging_controller::block_detect(void) const {
  return saa()->sensing()->env()->detect("block");
} /* block_detect() */

void foraging_controller::init(ticpp::Element& node) {
  /* verify environment variables set up for logging */
  ER_ENV_VERIFY();
  mdc_ts_update();

  ER_INFO("Initializing...");

  config::foraging_controller_repository repo;
  repo.parse_all(node);

  ndc_uuid_push();
  if (!repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize RNG */
  const auto* rngp = repo.config_get<rmath::config::rng_config>();
  base_controller2D::rng_init((nullptr == rngp) ? -1 : rngp->seed,
                              cpal::kRobotType);

  /* initialize output */
  base_controller2D::output_init(repo.config_get<cpconfig::output_config>());

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(repo.config_get<csubsystem::config::actuation_subsystem2D_config>(),
           repo.config_get<chal::subsystem::config::sensing_subsystemQ3D_config>());

  /* initialize supervisor */
  supervisor(std::make_unique<cfsm::supervisor_fsm>(saa()));

  /* initialize state trackers */
  inta_tracker(std::make_unique<cspatial::interference_tracker>(saa()->sensing()));
  m_nz_tracker = std::make_unique<cspatial::nest_zone_tracker>(saa()->sensing());

  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void foraging_controller::reset(void) { block_carrying_controller::reset(); }

fs::path foraging_controller::output_init(const cpconfig::output_config* outputp) {
  auto path =  base_controller2D::output_init(outputp);

#if (LIBRA_ER == LIBRA_ER_ALL)
  /*
   * Each file appender is attached to a root category in the COSM
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.ta"), path / "ta.log");

  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.controller"),
                 path / "controller.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.ds"), path / "ds.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.fsm"), path / "fsm.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.controller.saa"),
                 path / "saa.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.controller.explore_"
                                            "behavior"),
                 path / "saa.log");
#endif

  return path;
} /* output_init() */

void foraging_controller::saa_init(
    const csubsystem::config::actuation_subsystem2D_config* actuation_p,
    const chal::subsystem::config::sensing_subsystemQ3D_config* sensing_p) {
  auto actuators = actuation_init(this, actuation_p);
  auto sensors = sensing_init(this, sensing_p);

  auto saa = std::make_unique<csubsystem::saa_subsystemQ3D>(
      std::move(sensors),
      std::move(actuators),
      &actuation_p->steering);
  base_controller2D::saa(std::move(saa));
} /* saa_init() */


double foraging_controller::applied_movement_throttle(void) const {
  return saa()->actuation()->governed_diff_drive()->applied_throttle();
} /* applied_movement_throttle() */

void foraging_controller::irv_init(const ctv::robot_dynamics_applicator* rda) {
  if (rda->motion_throttling_enabled()) {
    saa()->actuation()->governed_diff_drive()->tv_generator(
        rda->motion_throttler(entity_id()));
  }
  if (rda->bc_throttling_enabled()) {
    saa()->actuation()->governed_diff_drive()->tv_generator(
        rda->bc_throttler(entity_id()));
  }
} /* irv_init() */

/*******************************************************************************
 * Movement Metrics
 ******************************************************************************/
rtypes::spatial_dist foraging_controller::ts_distance(
    const csmetrics::movement_category& category) const {
  if (csmetrics::movement_category::ekALL == category) {
    return ts_distance_impl();
  } else if (csmetrics::movement_category::ekHOMING == category) {
    if (fsm::foraging_transport_goal::ekNEST == block_transport_goal()) {
      return ts_distance_impl();
    }
  } else if (csmetrics::movement_category::ekEXPLORING == category) {
    auto status = is_exploring_for_goal();
    if (status.is_exploring && status.is_true) {
      return ts_distance_impl();
    }
  }
  return rtypes::spatial_dist(0);
} /* ts_distance() */

rmath::vector3d foraging_controller::ts_velocity(
    const csmetrics::movement_category& category) const {
  if (csmetrics::movement_category::ekALL == category) {
    return ts_velocity_impl();
  } else if (csmetrics::movement_category::ekHOMING == category) {
    if (fsm::foraging_transport_goal::ekNEST == block_transport_goal()) {
      return ts_velocity_impl();
    }
  } else if (csmetrics::movement_category::ekEXPLORING == category) {
    auto status = is_exploring_for_goal();
    if (status.is_exploring && status.is_true) {
      return ts_velocity_impl();
    }
  }
  return {};
} /* ts_velocity() */

/*******************************************************************************
 * Nest Zone Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEF(foraging_controller, in_nest, *m_nz_tracker, const)
RCPPSW_WRAP_DEF(foraging_controller, entered_nest, *m_nz_tracker, const);
RCPPSW_WRAP_DEF(foraging_controller, exited_nest, *m_nz_tracker, const);
RCPPSW_WRAP_DEF(foraging_controller, nest_duration, *m_nz_tracker, const);
RCPPSW_WRAP_DEF(foraging_controller, nest_entry_time, *m_nz_tracker, const);

NS_END(controller, fordyca);
