/**
 * \file dpo_metrics.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "cosm/repr/pheromone_density.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class dpo_metrics
 * \ingroup metrics perception
 *
 * \brief Defines the metrics to be collected from robots about their DPO world
 * model.
 *
 * Metrics are collected every timestep.
 */
class dpo_metrics : public virtual rmetrics::base_metrics {
 public:
  dpo_metrics(void) = default;

  /**
   * \brief Return the # of blocks that a robot currently knows about.
   */
  virtual size_t n_known_blocks(void) const = 0;

  /**
   * \brief Return The # of caches that a robot currently knows about.
   */
  virtual size_t n_known_caches(void) const = 0;

  /**
   * \brief Return the average block pheromone density for the blocks the robot
   * currently knows about.
   */
  virtual crepr::pheromone_density avg_block_density(void) const = 0;

  /**
   * \brief Return the average cache pheromone density for the caches the robot
   * currently knows about.
   */
  virtual crepr::pheromone_density avg_cache_density(void) const = 0;
};

NS_END(perception, metrics, fordyca);

