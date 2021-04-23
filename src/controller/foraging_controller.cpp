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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <filesystem>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"
#include "cosm/metrics/config/output_config.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/subsystem/config/sensing_subsystemQ3D_config.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"
#include "cosm/hal/subsystem/config/saa_xml_names.hpp"

#include "fordyca/config/foraging_controller_repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller::foraging_controller(void)
    : ER_CLIENT_INIT("fordyca.controller") {}

foraging_controller::~foraging_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool foraging_controller::in_nest(void) const {
  return saa()->sensing()->ground()->detect(
      chal::sensors::ground_sensor::kNestTarget);
} /* in_nest() */

bool foraging_controller::block_detected(void) const {
  return saa()->sensing()->ground()->detect("block");
} /* block_detected() */

void foraging_controller::init(ticpp::Element& node) {
  /* verify environment variables set up for logging */
  ER_ENV_VERIFY();

  config::foraging_controller_repository repo;
  repo.parse_all(node);

  ndc_push();
  if (!repo.validate_all()) {
    ER_FATAL_SENTINEL("Not all parameters were validated");
    std::exit(EXIT_FAILURE);
  }

  /* initialize RNG */
  const auto *rngp = repo.config_get<rmath::config::rng_config>();
  base_controller2D::rng_init((nullptr == rngp) ? -1 : rngp->seed,
                              cpal::kARGoSRobotType);

  /* initialize output */
  const auto * outputp = repo.config_get<cmconfig::output_config>();
  base_controller2D::output_init(outputp->output_root, outputp->output_dir);

  /* initialize sensing and actuation (SAA) subsystem */
  saa_init(repo.config_get<csubsystem::config::actuation_subsystem2D_config>(),
           repo.config_get<csubsystem::config::sensing_subsystemQ3D_config>());

  /* initialize supervisor */
  supervisor(std::make_unique<cfsm::supervisor_fsm>(saa()));
  ndc_pop();
} /* init() */

void foraging_controller::reset(void) { block_carrying_controller::reset(); }

void foraging_controller::output_init(const cmconfig::output_config* outputp) {
  std::string dir =
      base_controller2D::output_init(outputp->output_root, outputp->output_dir);

#if (LIBRA_ER == LIBRA_ER_ALL)
  /*
   * Each file appender is attached to a root category in the COSM
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.ta"), dir + "/ta.log");

  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.controller"),
                 dir + "/controller.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.ds"), dir + "/ds.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.fsm"), dir + "/fsm.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.controller.saa"),
                 dir + "/saa.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("fordyca.controller.explore_"
                                            "behavior"),
                 dir + "/saa.log");
#endif
} /* output_init() */

void foraging_controller::saa_init(
    const csubsystem::config::actuation_subsystem2D_config* actuation_p,
    const csubsystem::config::sensing_subsystemQ3D_config* sensing_p) {
  using saa_names = chsubsystem::config::saa_xml_names;

#if defined(FORDYCA_WITH_ROBOT_RAB) && (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
  auto rabs = chsensors::wifi_sensor(
      GetSensor<chsensors::wifi_sensor::impl_type>(saa_names::rab_saa));
#endif /* FORDYCA_WITH_ROBOT_RAB */

#if defined(FORDYCA_WITH_ROBOT_BATTERY) && (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
  auto battery = chsensors::battery_sensor(
      GetSensor<chsensors::battery_sensor::impl_type>(saa_names::battery_sensor));
#endif /* FORDYCA_WITH_ROBOT_BATTERY */

#ifdef FORDYCA_WITH_ROBOT_CAMERA
  auto blobs = chsensors::colored_blob_camera_sensor(
      GetSensor<chsensors::colored_blob_camera_sensor::impl_type>(
          saa_names::camera_sensor));
#else
  auto blobs = chsensors::colored_blob_camera_sensor(nullptr);
#endif
  auto position = chsensors::position_sensor(
      GetSensor<chsensors::position_sensor::impl_type>(saa_names::position_sensor));
  auto proximity = chsensors::proximity_sensor(
      GetSensor<chsensors::proximity_sensor::impl_type>(saa_names::prox_sensor),
      &sensing_p->proximity);
  auto light = chsensors::light_sensor(
      GetSensor<chsensors::light_sensor::impl_type>(saa_names::light_sensor));
  auto ground = chsensors::ground_sensor(
      GetSensor<chsensors::ground_sensor::impl_type>(saa_names::ground_sensor),
      &sensing_p->ground);

  auto diff_drives = chsensors::diff_drive_sensor(
      GetSensor<chsensors::diff_drive_sensor::impl_type>(
          saa_names::diff_steering_saa));

  auto sensors = csubsystem::sensing_subsystemQ3D::sensor_map{
#if defined(FORDYCA_WITH_ROBOT_RAB) && (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
    csubsystem::sensing_subsystemQ3D::map_entry_create(rabs),
#endif

#if defined(FORDYCA_WITH_ROBOT_BATTERY) && (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
    csubsystem::sensing_subsystemQ3D::map_entry_create(battery),
#endif
    csubsystem::sensing_subsystemQ3D::map_entry_create(proximity),
    csubsystem::sensing_subsystemQ3D::map_entry_create(blobs),
    csubsystem::sensing_subsystemQ3D::map_entry_create(light),
    csubsystem::sensing_subsystemQ3D::map_entry_create(ground),
    csubsystem::sensing_subsystemQ3D::map_entry_create(diff_drives),
    csubsystem::sensing_subsystemQ3D::map_entry_create(position)
  };

  auto diff_drivea = ckin2D::governed_diff_drive(
      &actuation_p->diff_drive,
      chactuators::diff_drive_actuator(
          GetActuator<chactuators::diff_drive_actuator::impl_type>(
              saa_names::diff_steering_saa)),
      ckin2D::governed_diff_drive::drive_type::ekFSM_DRIVE);

#ifdef FORDYCA_WITH_ROBOT_LEDS
  auto leds = chactuators::led_actuator(
      GetActuator<chactuators::led_actuator::impl_type>(saa_names::leds_saa));
#else
  auto leds = chactuators::led_actuator(nullptr);
#endif /* FORDYCA_WITH_ROBOT_LEDS */

#if defined(FORDYCA_WITH_ROBOT_RAB) && (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
  auto raba = chactuators::wifi_actuator(
      GetActuator<chactuators::wifi_actuator::impl_type>(saa_names::rab_saa));
#endif /* FORDYCA_WITH_ROBOT_RAB */

  auto actuators = csubsystem::actuation_subsystem2D::actuator_map{
    /*
     * We put the governed differential drive in the actuator map twice because
     * some of the reusable components use the base class differential drive
     * instead of the governed version (no robust way to inform that we want to
     * use the governed version).
     */
    csubsystem::actuation_subsystem2D::map_entry_create(diff_drivea),
    csubsystem::actuation_subsystem2D::map_entry_create(leds),

#if defined(FORDYCA_WITH_ROBOT_RAB) && (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
    csubsystem::actuation_subsystem2D::map_entry_create(raba)
#endif
  };

  base_controller2D::saa(std::make_unique<csubsystem::saa_subsystemQ3D>(sensors,
                                                                        actuators,
                                                                        &actuation_p->steering));
} /* saa_init() */

rtypes::type_uuid foraging_controller::entity_id(void) const {
  return rtypes::type_uuid(std::atoi(GetId().c_str() + 2));
} /* entity_id() */

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

NS_END(controller, fordyca);
