/**
 * @file site_selection_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_CACHES_SITE_SELECTION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_CACHES_SITE_SELECTION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <nlopt.hpp>

#include "rcppsw/metrics/base_metrics.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class site_selection_metrics
 * @ingroup fordyca metrics caches
 *
 * @brief Interface defining cache site selection metrics that can be collected
 * from robots choosing cache sites.
 */
class site_selection_metrics : public virtual rmetrics::base_metrics {
 public:
  site_selection_metrics(void) = default;
  ~site_selection_metrics(void) override = default;
  site_selection_metrics(const site_selection_metrics&) = default;
  site_selection_metrics& operator=(const site_selection_metrics&) = default;

  /**
   * @brief Return \c TRUE iff the robot has run the cache site selection
   * algorithm this timestep.
   */
  virtual bool site_select_exec(void) const = 0;

  /**
   * @brief Return \c TRUE iff the cache site selection algorithm was
   * successful.
   *
   * The result of this function is undefined if \ref site_select_exec() did
   * not return \c TRUE.
   */
  virtual bool site_select_success(void) const = 0;

  /**
   * @brief If the robot successfully selected a cache site, return the nlopt
   * code that led to successful selection.
   *
   * The result of this function is undefined if \ref site_selected() did not
   * return \c TRUE.
   */
  virtual nlopt::result nlopt_result(void) const = 0;
};

NS_END(caches, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_CACHES_SITE_SELECTION_METRICS_HPP_ */
