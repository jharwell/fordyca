/**
 * @file goal_acquisition_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_GOAL_ACQUISITION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_GOAL_ACQUISITION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector2.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

namespace rmath = rcppsw::math;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class goal_acquisition_metrics
 * @ingroup fordyca metrics fsm
 *
 * @brief Interface defining what metrics that should be collected from FSMs as
 * they attempt to acquire a goal (site/object of interest) in SOME way (driving
 * to it directly, exploring for it, etc).
 */
class goal_acquisition_metrics : public virtual rcppsw::metrics::base_metrics {
 public:
  enum class goal_type {
    kNone,
    kCacheSite,
    kNewCache,
    kExistingCache,
    kBlock
  };
  goal_acquisition_metrics(void) = default;
  ~goal_acquisition_metrics(void) override = default;

  /**
   * @brief Return the type of acquisition that is currently being
   * performed.
   *
   * @return The acquisition type, or \ref goal_type::kNone if no acquisition is
   * currently in progress.
   */
  virtual goal_type acquisition_goal(void) const = 0;

  /**
   * @brief Output only defined if \ref goal_type() is not \ref
   * goal_type::kNone. If \c TRUE, then the robot is currently exploring for its
   * goal (i.e. it does not know where it is).
   */
  virtual bool is_exploring_for_goal(void) const = 0;

  /**
   * @brief Output only defined if \ref goal_type() is not \ref
   * goal_type::kNone. If \c TRUE, then the robot is vectoring towards its goal
   * (i.e. it knows where it is).
   */
  virtual bool is_vectoring_to_goal(void) const = 0;

  /**
   * @brief If \c TRUE, then the robot has arrived at its goal, and is waiting
   * for some sort of signal from the simulation so that it can start executing
   * the next part of its current FSM as part of its current task.
   */
  virtual bool goal_acquired(void) const = 0;

  /**
   * @brief When \ref goal_acquired() returns \c TRUE, then this should return
   * the location of the goal that was acquired.
   */
  virtual rmath::vector2u acquisition_loc(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_GOAL_ACQUISITION_METRICS_HPP_ */
