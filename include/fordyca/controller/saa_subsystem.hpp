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
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "rcppsw/common/common.hpp"
#include "fordyca/controller/steering_force2D.hpp"
#include "rcppsw/robotics/kinematics/twist.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params {
struct actuation_params;
struct sensing_params;
} // namespace params

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class saa_subsystem
 * @ingroup controller
 *
 * @brief Sensing and Actuation subsystem for the robot. Does not do much other
 * than wrap the two components.
 */
class saa_subsystem : public rcppsw::robotics::steering2D::boid,
                      rcppsw::er::client {
 public:
  /**
   * @brief If our current heading and the applied steering force differ by less
   * than this amount, then a soft turn using the generated angular momentum is
   * used to gradually change heading while still moving forward. Above this
   * threshold an in-place hard-turn is used.
   */

  static constexpr double kSoftTurnThresh = M_PI/6;

  saa_subsystem(const std::shared_ptr<rcppsw::er::server>& server,
                const struct params::actuation_params* aparams,
                const struct params::sensing_params* sparams,
                struct actuation_subsystem::actuator_list* actuator_list,
                struct base_sensing_subsystem::sensor_list* sensor_list);

  /* BOID interface */
  argos::CVector2 linear_velocity(void) const override {
    double theta = m_sensing->robot_heading().Angle().GetValue();
    return argos::CVector2(m_actuation->differential_drive().current_speed() *
                           std::cos(theta),
                           m_actuation->differential_drive().current_speed() *
                           std::sin(theta));
  }
  double angular_velocity(void) const override {
    return (m_actuation->differential_drive().right_linspeed() -
            m_actuation->differential_drive().left_linspeed()) /
        m_actuation->differential_drive().axle_length();
  }
  double max_speed(void) const override {
    return m_actuation->differential_drive().max_speed();
  }
  argos::CVector2 position(void) const override {
    return m_sensing->position();
  }

  void sensing(const std::shared_ptr<base_sensing_subsystem>& sensing) {
    m_sensing = sensing;
  }

  /**
   * @brief Apply the summed steering forces; change wheel speeds. Resets the
   * summed forces.
   */
  void apply_steering_force(void);


  steering_force2D& steering_force(void) { return m_steering; }
  const steering_force2D& steering_force(void) const { return m_steering; }

  std::shared_ptr<base_sensing_subsystem> sensing(void) { return m_sensing; }
  const std::shared_ptr<const base_sensing_subsystem> sensing(void) const {
    return m_sensing;
  }

  const std::shared_ptr<const actuation_subsystem> actuation(void) const {
    return m_actuation;
  }
  std::shared_ptr<controller::actuation_subsystem> actuation(void) {
    return m_actuation;
  }

 private:
  // clang-format off
  std::shared_ptr<controller::actuation_subsystem> m_actuation;
  std::shared_ptr<base_sensing_subsystem>          m_sensing;
  steering_force2D                                 m_steering;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_SAA_SUBSYSTEM_HPP_ */
