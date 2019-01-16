/**
 * @file base_perception_subsystem.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_BASE_PERCEPTION_SUBSYSTEM_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BASE_PERCEPTION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/representation/line_of_sight.hpp"
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_perception_subsystem
 * @ingroup controller
 */
class base_perception_subsystem {
 public:
  base_perception_subsystem(void) = default;
  virtual ~base_perception_subsystem(void) = default;

  /**
   * @brief Reset the robot's perception of the environment to an initial state
   */
  virtual void reset(void) {}

  /**
   * @brief Update the internal data structure/representation of the
   * environment/arena, after the LOS has been updated.
   */
  virtual void update(void) = 0;

  /**
   * @brief Set the robots LOS for the next timestep.
   *
   * This is a hack to make it easy for me to run simulations, as I can computer
   * the line of sight for a robot within the loop functions, and just pass it
   * in here. In real robots this routine would be MUCH messier and harder to
   * work with.
   *
   * @param los The new los
   */
  void los(std::unique_ptr<representation::line_of_sight> los) {
    m_los = std::move(los);
  }

  /**
   * @brief Get the robot's current line-of-sight (LOS)
   */
  const representation::line_of_sight* los(void) const { return m_los.get(); }

 private:
  /* clang-format off */
  std::unique_ptr<representation::line_of_sight> m_los{nullptr};
  /* clang-format on */
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BASE_PERCEPTION_SUBSYSTEM_HPP_ */
