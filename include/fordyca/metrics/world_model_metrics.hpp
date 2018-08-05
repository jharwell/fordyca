/**
 * @file world_model_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_WORLD_MODEL_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_WORLD_MODEL_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class world_model_metrics
 * @ingroup metrics
 *
 * @brief Defines the metrics to be collected from robots about their world
 * model.
 *
 * Metrics are collected every timestep.
 */
class world_model_metrics : virtual public rcppsw::metrics::base_metrics {
 public:
  world_model_metrics(void) = default;

  /**
   * @brief Return the total # of times the robot's world model was inaccurate
   * regarding which cells in their world model were thought to contain
   * something/not to contain something.
   *
   * This does not collect any information about WHERE in the world model an
   * inaccuracy was detected, only that there was one. That may be added later
   * if additional insight is desired.
   *
   * @param state The state type for which inaccuracies should be reported.
   */
  virtual uint cell_state_inaccuracies(uint state) const = 0;
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_WORLD_MODEL_METRICS_HPP_ */
