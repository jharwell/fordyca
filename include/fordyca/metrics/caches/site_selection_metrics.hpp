/**
 * \file site_selection_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

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
 * \class site_selection_metrics
 * \ingroup metrics caches
 *
 * \brief Interface defining cache site selection metrics that can be collected
 * from robots choosing cache sites.
 */
class site_selection_metrics : public virtual rmetrics::base_metrics {
 public:
  site_selection_metrics(void) = default;
  ~site_selection_metrics(void) override = default;
  site_selection_metrics(const site_selection_metrics&) = default;
  site_selection_metrics& operator=(const site_selection_metrics&) = default;

  /**
   * \brief Return \c TRUE iff the robot has run the cache site selection
   * algorithm this timestep.
   */
  virtual bool site_select_exec(void) const = 0;

  /**
   * \brief Return \c TRUE iff the cache site selection algorithm was
   * successful.
   *
   * The result of this function is undefined if \ref site_select_exec() did
   * not return \c TRUE.
   */
  virtual bool site_select_success(void) const = 0;

  /**
   * \brief If the robot successfully selected a cache site, return the nlopt
   * code that led to successful selection.
   *
   * The result of this function is undefined if \ref site_select_exec() did not
   * return \c TRUE.
   */
  virtual nlopt::result nlopt_result(void) const = 0;
};

NS_END(caches, metrics, fordyca);

