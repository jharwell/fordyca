/**
 * @file block_acquisition_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_BLOCK_ACQUISITION_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_BLOCK_ACQUISITION_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/fsm/base_fsm_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_acquisition_metrics
 * @ingroup metrics fsm
 *
 * @brief Interface defining what metrics should be collected FSMs as they
 * attempt to acquire a block from SOMEWHERE in SOME way.
 */
class block_acquisition_metrics : public base_fsm_metrics {
 public:
  block_acquisition_metrics(void) = default;
  ~block_acquisition_metrics(void) override = default;

  /**
   * @brief If \c TRUE, then a robot is currently exploring for a block (no
   * known blocks) via the \ref explore_for_block_fsm.
   */
  virtual bool is_exploring_for_block(void) const = 0;

  /**
   * @brief If \c TRUE, then a robot is currently acquiring a block (either via
   * exploring or via vectoring)
   */
  virtual bool is_acquiring_block(void) const = 0;

  /**
   * @brief If \c TRUE, then a robot is currently acquiring a block via
   * vectoring via the \ref vector_fsm.
   */
  virtual bool is_vectoring_to_block(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_BLOCK_ACQUISITION_METRICS_HPP_ */
