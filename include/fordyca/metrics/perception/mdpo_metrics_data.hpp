/**
 * \file mdpo_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_PERCEPTION_MDPO_METRICS_DATA_HPP_
#define INCLUDE_FORDYCA_METRICS_PERCEPTION_MDPO_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>
#include <array>

#include "rcppsw/metrics/base_metrics_data.hpp"
#include "cosm/fsm/cell2D_state.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, perception, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct mdpo_metrics_data
 * \ingroup metrics perception detail
 *
 * \brief Container for holding collected statistics of \ref
 * mdpo_metrics. Must be atomic so counts are valid in parallel metric
 * collection contexts.
 */
struct mdpo_metrics_data {
  std::array<std::atomic_uint, cfsm::cell2D_state::ekST_MAX_STATES> states;
  std::atomic<double> known_percent{0.0};
  std::atomic<double> unknown_percent{0.0};
  std::atomic_uint    robots{0};
};

NS_END(detail);

struct mdpo_metrics_data : public rmetrics::base_metrics_data {
  detail::mdpo_metrics_data interval{};
  detail::mdpo_metrics_data cum{};
};

NS_END(perception, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_PERCEPTION_MDPO_METRICS_DATA_HPP_ */
