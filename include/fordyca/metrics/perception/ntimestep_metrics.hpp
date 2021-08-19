/**
 * \file ntimestep_perception_metrics.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_PERCEPTION_NTIMESTEP_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_PERCEPTION_NTIMESTEP_METRICS_HPP_

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
 * \class ntimestep_metrics
 * \ingroup metrics perception
 *
 * \brief Defines the metrics to be collected from robots about their DPO world
 * model.
 *
 * Metrics are collected every timestep.
 */
class ntimestep_metrics : public virtual rmetrics::base_metrics {
 public:
  ntimestep_metrics(void) = default;

  /**
   * \brief Return the # of blocks that a robot currently knows about.
   */
  virtual uint n_known_blocks(void) const = 0;

  /**
   * \brief Return The # of caches that a robot currently knows about.
   */
  virtual uint n_known_caches(void) const = 0;

};

NS_END(perception, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_PERCEPTION_NTIMESTEP_METRICS_HPP_ */
