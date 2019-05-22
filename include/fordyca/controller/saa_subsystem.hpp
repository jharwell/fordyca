/**
 * @file saa_subsystem.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_SAA_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_SAA_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <memory>

#include "fordyca/controller/steering_force2D.hpp"
#include "fordyca/nsalias.hpp"
#include "fordyca/controller/sensor_list.hpp"
#include "fordyca/controller/actuator_list.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace config {
struct actuation_config;
struct sensing_config;
} // namespace config

NS_START(controller);
class actuation_subsystem;
class sensing_subsystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class saa_subsystem
 * @ingroup fordyca controller
 *
 * @brief Sensing and Actuation subsystem for the robot. Does not do much other
 * than wrap the two components.
 */
class saa_subsystem final : public rcppsw::robotics::steer2D::boid,
                            rer::client<saa_subsystem> {
 public:
  saa_subsystem(const config::actuation_config* aconfig,
                const config::sensing_config* sconfig,
                actuator_list* actuator_list,
                sensor_list* sensor_list);

  /* BOID interface */
  rmath::vector2d linear_velocity(void) const override;
  double angular_velocity(void) const override;
  double max_speed(void) const override;
  rmath::vector2d position(void) const override;

  void sensing(const std::shared_ptr<sensing_subsystem>& sensing) {
    m_sensing = sensing;
  }

  /**
   * @brief Apply the summed steering forces; change wheel speeds. Resets the
   * summed forces.
   */
  void apply_steering_force(const std::pair<bool, bool>& force);

  steering_force2D& steering_force(void) { return m_steering; }
  const steering_force2D& steering_force(void) const { return m_steering; }

  std::shared_ptr<sensing_subsystem> sensing(void) { return m_sensing; }
  const std::shared_ptr<const sensing_subsystem> sensing(void) const {
    return m_sensing;
  }

  const std::shared_ptr<const actuation_subsystem> actuation(void) const {
    return m_actuation;
  }
  std::shared_ptr<controller::actuation_subsystem> actuation(void) {
    return m_actuation;
  }

 private:
  /* clang-format off */
  std::shared_ptr<controller::actuation_subsystem> m_actuation;
  std::shared_ptr<sensing_subsystem>               m_sensing;
  steering_force2D                                 m_steering;
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_SAA_SUBSYSTEM_HPP_ */
