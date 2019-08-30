/**
 * @file manipulation_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_BLOCKS_MANIPULATION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_BLOCKS_MANIPULATION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "fordyca/fordyca.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class manipulation_metrics
 * @ingroup fordyca metrics blocks
 *
 * @brief Defines the metrics to be collected from blocks about the process of
 * manipulation (pickup, drop, etc.)
 */
class manipulation_metrics : public virtual rmetrics::base_metrics {
 public:
  manipulation_metrics(void) = default;
  ~manipulation_metrics(void) override = default;

  /**
   * @brief If \c TRUE, then a free block was picked up this timestep.
   */
  virtual bool free_pickup_event(void) const = 0;

  /**
   * @brief If \c TRUE, then a block was dropped in the arena or in the nest
   * this timestep.
   */
  virtual bool free_drop_event(void) const = 0;

  /**
   * @brief The penalty the robot was subjected to when picking up/dropping a
   * block. Only valid for the specific manipulation type, as determined by
   * calling the other members of the class.
   */
  virtual rtypes::timestep penalty_served(void) const = 0;

  /**
   * @brief If \c TRUE, then a cached block was picked up this timestep.
   */
  virtual bool cache_pickup_event(void) const = 0;

  /**
   * @brief If \c TRUE, then a block was dropped in a cache this timestep by the
   * robot.
   */
  virtual bool cache_drop_event(void) const = 0;
};

NS_END(blocks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BLOCKS_MANIPULATION_METRICS_HPP_ */
