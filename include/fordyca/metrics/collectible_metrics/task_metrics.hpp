/**
 * @file task_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_TASK_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_TASK_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectible_metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class task_metrics
 *
 * @brief Interface defining metrics that can be collected on running tasks.
 */
class task_metrics {
 public:
  task_metrics(void) {}
  virtual ~task_metrics(void) {}

  /**
   * @brief Get the name of the name that is currently being executed
   */
  virtual std::string task_name(void) const = 0;
};

NS_END(collectible_metrics, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_COLLECTIBLE_METRICS_TASK_METRICS_HPP_ */
