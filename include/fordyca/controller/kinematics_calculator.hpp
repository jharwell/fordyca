/**
 * @file kinematics_calculator.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_KINEMATICS_CALCULATOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_KINEMATICS_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

class base_foraging_sensors;
class actuator_manager;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class kinematics_calculator
 * @ingroup controller
 *
 * @brief Calculate various attractive/repulsive forces acting on the robot at
 * any given time, such as
 *
 * - Obstacle avoidance
 * - light attraction
 * - light repulsion
 */
class kinematics_calculator {
 public:
  kinematics_calculator(std::shared_ptr<base_foraging_sensors>& sensors,
                        std::shared_ptr<actuator_manager>& actuators)
      : mc_actuators(actuators),
        m_sensors(sensors) {}

  /**
   * @brief Calculates the force applied to a robot as it nears an obstacle.
   *
   * Since robots have only a proximity sensor which returns something in [0, 1]
   * indication how close something is, NOT how far it actually is away from the
   * robot, I'm not able to do the usual lookahead/prediction using current
   * velocity to smoothly navigate around obstacles. The result is not terrible,
   * but it could be better.
   */
  argos::CVector2 calc_avoidance_force(void);

  /**
   * @brief Calculates the attraction to the light source. Used to perform
   * phototaxis.
   */
  argos::CVector2 calc_light_attract_force(void);

  /**
   * @brief Calculates the repulsion from the light source. Used to perform
   * antiphototaxis.
   */
  argos::CVector2 calc_light_repel_force(void);

 private:
  static constexpr double kSCALE_LIGHT_FORCE_ATTRACT = 0.8;
  static constexpr double kSCALE_LIGHT_FORCE_REPEL = 0.5;
  static constexpr double kSCALE_AVOIDANCE_FORCE = 0.5;

  std::shared_ptr<actuator_manager> mc_actuators;
  std::shared_ptr<base_foraging_sensors> m_sensors;
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_KINEMATICS_CALCULATOR_HPP_ */
