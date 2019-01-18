/**
 * @file throttling_differential_drive.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_THROTTLING_DIFFERENTIAL_DRIVE_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_THROTTLING_DIFFERENTIAL_DRIVE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/tv/motion_throttling_handler.hpp"
#include "rcppsw/common/common.hpp"
#include "rcppsw/robotics/hal/actuators/differential_drive_actuator.hpp"
#include "rcppsw/robotics/kinematics2D/differential_drive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

namespace kinematics2D = rcppsw::robotics::kinematics2D;
namespace hal = rcppsw::robotics::hal;
namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class throttling_differential_drive : public kinematics2D::differential_drive {
 public:
  static constexpr double kWheelRadius = 0.029112741;
  static constexpr double kInterWheelDistance = 0.14;

  throttling_differential_drive(
      kinematics2D::differential_drive::drive_type type,
      double max_speed,
      const rmath::radians& soft_turn_max,
      const hal::actuators::differential_drive_actuator& wheels)
      : differential_drive(wheels,
                           type,
                           kWheelRadius,
                           kInterWheelDistance,
                           max_speed,
                           soft_turn_max) {}

  throttling_differential_drive(const throttling_differential_drive& other) =
      delete;
  const throttling_differential_drive& operator=(
      const throttling_differential_drive& other) = delete;

  double active_throttle(void) const {
    return mc_throttling->active_throttle();
  }
  double applied_throttle(void) const {
    return mc_throttling->applied_throttle();
  }

  void throttling(const support::tv::motion_throttling_handler* throttling) {
    mc_throttling = throttling;
  }

 private:
  /* clang-format off */
  const support::tv::motion_throttling_handler* mc_throttling{nullptr};
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_THROTTLING_DIFFERENTIAL_DRIVE_HPP_ */
