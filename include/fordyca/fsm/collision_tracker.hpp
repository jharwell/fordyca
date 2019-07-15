/**
 * @file collision_tracker.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_FSM_COLLISION_TRACKER_HPP_
#define INCLUDE_FORDYCA_FSM_COLLISION_TRACKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/fsm/collision_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);
namespace controller {
class saa_subsystem;
} /* namespace controller */

NS_START(fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class collision_tracker : public metrics::fsm::collision_metrics {
 public:
  explicit collision_tracker(const controller::saa_subsystem* const saa)
      : mc_saa(saa) {}
  collision_tracker(const collision_tracker&) = delete;
  collision_tracker& operator=(const collision_tracker&) = delete;

  /* collision metrics */
  bool in_collision_avoidance(void) const override final RCSW_PURE;
  bool entered_collision_avoidance(void) const override final RCSW_PURE;
  bool exited_collision_avoidance(void) const override final RCSW_PURE;
  rtypes::timestep collision_avoidance_duration(void) const override final;
  rmath::vector2u avoidance_loc(void) const override final RCSW_PURE;

  /**
   * @brief Handle all logic for entering collision avoidance; classes should
   * only have to call this function whenever they detect an obstacle.
   */
  void ca_enter(void);

  /**
   * @brief Handle all logic for exiting collision avoidance; classes should
   * only have to call this function whenever they no longer detect any
   * obstacles.
   */
  void ca_exit(void);

 private:
  /* clang-format off */
  bool                                   m_task_running{false};
  bool                                   m_entered_avoidance{false};
  bool                                   m_exited_avoidance{false};
  bool                                   m_in_avoidance{false};
  rtypes::timestep                       m_avoidance_start{0};
  const controller::saa_subsystem* const mc_saa;
  /* clang-format on */
};

NS_END(fsm, fordyca);

#endif /* INCLUDE_FORDYCA_FSM_COLLISION_TRACKER_HPP_ */
