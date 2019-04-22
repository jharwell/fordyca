/**
 * @file transport_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_BLOCKS_TRANSPORT_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_BLOCKS_TRANSPORT_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

namespace rmetrics = rcppsw::metrics;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class transport_metrics
 * @ingroup metrics blocks
 *
 * @brief Defines the metrics to be collected from blocks about the process of
 * transportation from their original location in the arena after distribution
 * to the nest.
 *
 * Metrics should be collected upon deposition in nest, rather than every
 * timestep.
 */
class transport_metrics : public rmetrics::base_metrics {
 public:
  enum block_type {
    kCube,
    kRamp
  };

  transport_metrics(void) = default;

  /**
   * @brief Return the total # of robots that have carried the block since it
   * was originally distributed in the arena until it makes it all the way back
   * to the nest.
   */
  virtual uint total_transporters(void) const = 0;

  /**
   * @brief Return the total amount of time that it took from the first pickup
   * to when the block was deposited in the nest.
   */
  virtual double total_transport_time(void) const = 0;

  /**
   * @brief Return the amount of time that the block sits in the arena after
   * being distributed but before it is picked up for the first time.
   */
  virtual double initial_wait_time(void) const = 0;

  /**
   * @brief When a block is collected, return the type of the block
   */
  virtual block_type type(void) const = 0;
};

NS_END(blocks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BLOCKS_TRANSPORT_METRICS_HPP_ */
