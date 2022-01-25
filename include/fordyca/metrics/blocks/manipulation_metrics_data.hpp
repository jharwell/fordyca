/**
 * \file manipulation_metrics_data.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_BLOCKS_MANIPULATION_METRICS_DATA_HPP_
#define INCLUDE_FORDYCA_METRICS_BLOCKS_MANIPULATION_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>
#include <array>

#include "rcppsw/metrics/base_data.hpp"
#include "fordyca/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, metrics, blocks, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct manipulation_metrics_data
 * \ingroup metrics blocks detail
 *
 * \brief Container for holding collected statistics of \ref
 * manipulation_metrics. Must be atomic so counts are valid in parallel metric
 * collection contexts.
 */
struct manipulation_metrics_data {
  std::atomic_size_t events{0};
  std::atomic_size_t penalties{0};
};

NS_END(detail);

struct manipulation_metrics_data : public rmetrics::base_data {
  using array_type = std::array<detail::manipulation_metrics_data,
                                block_manip_events::ekMAX_EVENTS>;
  array_type interval{};
  array_type cum{};
};

NS_END(blocks, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_BLOCKS_MANIPULATION_METRICS_DATA_HPP_ */
