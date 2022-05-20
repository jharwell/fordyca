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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <array>

#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

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
  ral::mt_size_t events{0};
  ral::mt_size_t penalties{0};
};

NS_END(detail);

struct manipulation_metrics_data : public rmetrics::base_data {
  using array_type = std::array<detail::manipulation_metrics_data,
                                block_manip_events::ekMAX_EVENTS>;
  array_type interval{};
  array_type cum{};

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  manipulation_metrics_data& operator+=(const manipulation_metrics_data& rhs) {
    for (size_t i = 0; i < fmblocks::block_manip_events::ekMAX_EVENTS; ++i) {
      ral::mt_accum(this->interval[i].events, rhs.interval[i].events);
      ral::mt_accum(this->interval[i].penalties, rhs.interval[i].penalties);

      ral::mt_accum(this->cum[i].events, rhs.interval[i].events);
      ral::mt_accum(this->cum[i].penalties, rhs.interval[i].penalties);
    } /* for(i..) */
    return *this;
  }
};

NS_END(blocks, metrics, fordyca);
