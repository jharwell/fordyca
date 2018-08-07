/**
 * @file random_explore_behavior.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_RANDOM_EXPLORE_BEHAVIOR_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_RANDOM_EXPLORE_BEHAVIOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/explore_behavior.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class random_explore_behavior
 * @ingroup controller
 *
 *
 * @brief Perform random walk exploration: wander force + avoidance force.
 */
class random_explore_behavior : public explore_behavior {
 public:
  random_explore_behavior(std::shared_ptr<rcppsw::er::server> server,
                          controller::saa_subsystem* saa);

  ~random_explore_behavior(void) override = default;
  random_explore_behavior(const random_explore_behavior& fsm) = delete;
  random_explore_behavior& operator=(const random_explore_behavior& fsm) = delete;

  void execute(void) override;

  /* collision metrics */
  bool in_collision_avoidance(void) const override;
  bool entered_collision_avoidance(void) const override;
  bool exited_collision_avoidance(void) const override;
  uint collision_avoidance_duration(void) const override;

 private:
  // clang-format off
  bool m_entered_avoidance{false};
  bool m_exited_avoidance{false};
  bool m_in_avoidance{false};
  uint m_avoidance_start{0};
  // clang-format on
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_RANDOM_EXPLORE_BEHAVIOR_HPP_ */
