/**
 * @file block_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_BLOCK_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_BLOCK_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, collectible_metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_metrics
 * @ingroup metrics
 *
 * @brief Interface defining collectible metrics on a block.
 */
class block_metrics : public rcppsw::metrics::base_metrics {
 public:
  block_metrics(void) = default;
  ~block_metrics(void) override = default;
  block_metrics(const block_metrics&) = default;
  block_metrics& operator=(const block_metrics&) = default;

  /**
   * @brief Get how many carries this object has had on its way from its
   * original arena location back to the nest.
   *
   * @return # carries.
   */
  virtual size_t n_carries(void) const = 0;
};

NS_END(collectible_metrics, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BLOCK_METRICS_HPP_ */
