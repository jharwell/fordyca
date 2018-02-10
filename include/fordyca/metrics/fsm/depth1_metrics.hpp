/**
 * @file depth1_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_DEPTH1_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_DEPTH1_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class depth1_metrics
 * @ingroup metrics fsm
 *
 * @brief Interface defining what metrics that should be collected from robots
 * executing the depth1 foraging controller.
 */
class depth1_metrics : public rcppsw::metrics::base_metrics {
 public:
  depth1_metrics(void) = default;
  ~depth1_metrics(void) override = default;

  /**
   * @brief If \c TRUE, then the robot is currently running the
   * \ref explore_for_cache_fsm.
   */
  virtual bool is_exploring_for_cache(void) const = 0;

  /**
   * @brief If \c TRUE, then the robot is currently running the
   * \ref vector_fsm and traveling toward a known cache.
   */
  virtual bool is_vectoring_to_cache(void) const = 0;

  /**
   * @brief If \c TRUE, then the robot is currently running the
   * \ref acquire_cache_fsm, and is acquiring a cache either through exploring
   * or by vectoring to a known one.
   */
  virtual bool is_acquiring_cache(void) const = 0;

  /**
   * @brief If \c TRUE, then the robot is currently running the
   * \ref block_to_cache_fsm, and is transporting an acquired block to its cache
   * of choice.
   */
  virtual bool is_transporting_to_cache(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_DEPTH1_METRICS_HPP_ */
