/**
 * @file cache_acquisition_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_CACHE_ACQUISITION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_CACHE_ACQUISITION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/fsm/base_fsm_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class cache_acquisition_metrics
 * @ingroup metrics fsm
 *
 * @brief Interface defining what metrics that should be collected from FSMs as
 * they attempt to acquire a cache from SOMEWHERE in SOME way.
 */
class cache_acquisition_metrics : public base_fsm_metrics {
 public:
  cache_acquisition_metrics(void) = default;
  ~cache_acquisition_metrics(void) override = default;

  /**
   * @brief If \c TRUE, then the cache the robot is acquiring already exists (or
   * at least the robot thinks it does).
   */
  virtual bool acquisition_exists(void) const = 0;

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
   * @brief If \c TRUE, then the robot has arrived at a cache, and is waiting
   * for the simulation to send it the block pickup/drop signal.
   */
  virtual bool cache_acquired(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_CACHE_ACQUISITION_METRICS_HPP_ */
