/**
 * \file manipulation_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/blocks/manipulation_metrics_collector.hpp"

#include "cosm/controller/metrics/manipulation_metrics.hpp"

#include "fordyca/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
manipulation_metrics_collector::manipulation_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
manipulation_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_free_pickup_events",
    "int_avg_free_drop_events",
    "int_avg_free_pickup_penalty",
    "int_avg_free_drop_penalty",
    "int_avg_cache_pickup_events",
    "int_avg_cache_drop_events",
    "int_avg_cache_pickup_penalty",
    "int_avg_cache_drop_penalty",

    "cum_avg_free_pickup_events",
    "cum_avg_free_drop_events",
    "cum_avg_free_pickup_penalty",
    "cum_avg_free_drop_penalty",
    "cum_avg_cache_pickup_events",
    "cum_avg_cache_drop_events",
    "cum_avg_cache_pickup_penalty",
    "cum_avg_cache_drop_penalty"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void manipulation_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string>
manipulation_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0UL)) {
    return boost::none;
  }
  std::string line;

  /* interval averages */
  line += csv_entry_intavg(m_interval[block_manip_events::ekFREE_PICKUP].events);
  line += csv_entry_intavg(m_interval[block_manip_events::ekFREE_DROP].events);

  line +=
      csv_entry_domavg(m_interval[block_manip_events::ekFREE_PICKUP].penalties,
                       m_interval[block_manip_events::ekFREE_PICKUP].events);
  line += csv_entry_domavg(m_interval[block_manip_events::ekFREE_DROP].penalties,
                           m_interval[block_manip_events::ekFREE_DROP].events);

  line += csv_entry_intavg(m_interval[block_manip_events::ekCACHE_PICKUP].events);
  line += csv_entry_intavg(m_interval[block_manip_events::ekCACHE_DROP].events);

  line +=
      csv_entry_domavg(m_interval[block_manip_events::ekCACHE_PICKUP].penalties,
                       m_interval[block_manip_events::ekCACHE_PICKUP].events);
  line += csv_entry_domavg(m_interval[block_manip_events::ekCACHE_DROP].penalties,
                           m_interval[block_manip_events::ekCACHE_DROP].events);

  /* cumulative averages */
  line += csv_entry_tsavg(m_cum[block_manip_events::ekFREE_PICKUP].events);
  line += csv_entry_tsavg(m_cum[block_manip_events::ekFREE_DROP].events);

  line += csv_entry_domavg(m_cum[block_manip_events::ekFREE_PICKUP].penalties,
                           m_cum[block_manip_events::ekFREE_PICKUP].events);
  line += csv_entry_domavg(m_cum[block_manip_events::ekFREE_DROP].penalties,
                           m_cum[block_manip_events::ekFREE_DROP].events);

  line += csv_entry_tsavg(m_cum[block_manip_events::ekCACHE_PICKUP].events);
  line += csv_entry_tsavg(m_cum[block_manip_events::ekCACHE_DROP].events);

  line += csv_entry_domavg(m_cum[block_manip_events::ekCACHE_PICKUP].penalties,
                           m_cum[block_manip_events::ekCACHE_PICKUP].events);
  line += csv_entry_domavg(m_cum[block_manip_events::ekCACHE_DROP].penalties,
                           m_cum[block_manip_events::ekCACHE_DROP].events,
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */

void manipulation_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  const auto& m = dynamic_cast<const ccmetrics::manipulation_metrics&>(metrics);

  for (uint i = 0; i < block_manip_events::ekMAX_EVENTS; ++i) {
    m_interval[i].events += m.status(i);
    m_interval[i].penalties += m.penalty(i).v();

    m_cum[i].events += m.status(i);
    m_cum[i].penalties += m.penalty(i).v();
  } /* for(i..) */
} /* collect() */

void manipulation_metrics_collector::reset_after_interval(void) {
  for (auto& e : m_interval) {
    std::atomic_init(&e.events, 0UL);
    std::atomic_init(&e.penalties, 0UL);
  } /* for(e..) */
} /* reset_after_interval() */

NS_END(blocks, metrics, fordyca);
