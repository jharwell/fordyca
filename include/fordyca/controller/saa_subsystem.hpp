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
#include "rcppsw/common/common.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "rcppsw/control/kinematics2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace params { struct actuation_params; struct sensing_params; }

NS_START(controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

class saa_subsystem : public rcppsw::control::boid {
 public:
  saa_subsystem(
      const struct params::actuation_params* aparams,
      const struct params::sensing_params* sparams,
      struct actuation_subsystem::actuator_list* actuator_list,
      struct base_sensing_subsystem::sensor_list* sensor_list);

  /* BOID interface */
  argos::CVector2 velocity(void) const override {
    return m_sensing->robot_heading().Normalize().Scale(
        m_actuation->differential_drive()->current_speed(),
        m_actuation->differential_drive()->current_speed());
  }

  void sensing(const std::shared_ptr<base_sensing_subsystem>& sensing) { m_sensing = sensing; }

  double max_velocity(void) const override {
    return m_actuation->differential_drive()->max_speed();
  }
  argos::CVector2 position(void) const override { return m_sensing->position(); }

  const control::kinematics2D& kinematics(void) const { return m_kinematics; }
  control::kinematics2D& kinematics(void) { return m_kinematics; }

  std::shared_ptr<const base_sensing_subsystem> sensing(void) const { return m_sensing; }
  std::shared_ptr<base_sensing_subsystem> sensing(void) { return m_sensing; }

  std::shared_ptr<const actuation_subsystem> actuation(void) const { return m_actuation; }
  std::shared_ptr<controller::actuation_subsystem> actuation(void) { return m_actuation; }

 private:
  // clang-format off
  control::kinematics2D                           m_kinematics;
  std::shared_ptr<controller::actuation_subsystem> m_actuation;
  std::shared_ptr<base_sensing_subsystem>         m_sensing;
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_SAA_SUBSYSTEM_HPP_ */
