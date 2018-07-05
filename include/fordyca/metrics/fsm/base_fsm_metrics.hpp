/**
 * @file base_fsm_metrics.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FSM_BASE_FSM_METRICS_HPP_
#define INCLUDE_FORDYCA_METRICS_FSM_BASE_FSM_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class base_fsm_metrics
 * @ingroup metrics fsm
 *
 * @brief Interface defining what metrics should be collected ANY FSM as it goes
 * about whatever task it is supposed to do.
 */
class base_fsm_metrics : public virtual rcppsw::metrics::base_metrics {
 public:
  base_fsm_metrics(void) = default;
  ~base_fsm_metrics(void) override = default;

  /**
   * @brief If \c TRUE, then a robot is currently engaged in collision avoidance.
   */
  virtual bool is_avoiding_collision(void) const = 0;
};

NS_END(fsm, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FSM_BASE_FSM_METRICS_HPP_ */
