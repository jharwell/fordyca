/**
 * \file mdpo_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class mdpo_metrics
 * \ingroup metrics perception
 *
 * \brief Defines the metrics to be collected from robots about their mapped DPO
 * world model.
 *
 * Metrics are collected every timestep.
 */
class mdpo_metrics : public virtual rmetrics::base_metrics {
 public:
  mdpo_metrics(void) = default;

  /**
   * \brief Return the total # of times the robot's world model was inaccurate
   * regarding which cells in their world model were thought to contain
   * something/not to contain something.
   *
   * This does not collect any information about WHERE in the world model an
   * inaccuracy was detected, only that there was one. That may be added later
   * if additional insight is desired.
   *
   * \param state The state type for which inaccuracies should be reported.
   */
  virtual uint cell_state_inaccuracies(uint state) const = 0;

  /**
   * \brief Return the percentage of the arena that is currently in a known
   * state as a fraction of 1.0.
   */
  virtual double known_percentage(void) const = 0;

  /**
   * \brief Return the percentage of the arena that is currently in an unknown
   * state, as a fraction of 1.0
   */
  virtual double unknown_percentage(void) const = 0;
};

NS_END(perception, metrics, fordyca);

