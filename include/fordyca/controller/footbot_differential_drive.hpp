/**
 * @file footbot_differential_drive.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_FOOTBOT_DIFFERENTIAL_DRIVE_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_FOOTBOT_DIFFERENTIAL_DRIVE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include "rcppsw/common/common.hpp"
#include "rcppsw/robotics/kinematics2D/differential_drive.hpp"
#include "fordyca/controller/throttling_handler.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);
namespace kinematics2D = rcppsw::robotics::kinematics2D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class footbot_differential_drive : public kinematics2D::differential_drive {
 public:
  static constexpr double kWheelRadius = 0.029112741;
  static constexpr double kInterWheelDistance = 0.14;

  footbot_differential_drive(const std::shared_ptr<rcppsw::er::server>& server,
                             kinematics2D::differential_drive::drive_type type,
                             double max_speed,
                             argos::CRadians soft_turn_max,
                             argos::CCI_DifferentialSteeringActuator* wheels,
                             const throttling_handler& handler)
      : differential_drive(server,
                           type,
                           kWheelRadius,
                           kInterWheelDistance,
                           max_speed,
                           soft_turn_max),
        mc_throttling(handler),
        m_wheels(wheels) {}

  footbot_differential_drive(const footbot_differential_drive& fsm) = delete;
  footbot_differential_drive& operator=(const footbot_differential_drive& fsm) = delete;

  void set_wheel_speeds(double left, double right) override {
    m_wheels->SetLinearVelocity((1.0 - mc_throttling.block_carry()) * left,
                                (1.0 - mc_throttling.block_carry()) * right);
  }

 private:
  const throttling_handler& mc_throttling;
  argos::CCI_DifferentialSteeringActuator* m_wheels;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_FOOTBOT_DIFFERENTIAL_DRIVE_HPP_ */
