/**
 * \file manipulation_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/blocks/manipulation_metrics_csv_sink.hpp"

#include "fordyca/metrics/blocks/manipulation_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
manipulation_metrics_csv_sink::manipulation_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : ER_CLIENT_INIT("fordyca.metrics.blocks.manipulation_metrics_csv_sink"),
      csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
manipulation_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
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

boost::optional<std::string>
manipulation_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                              const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  std::string line;
  auto* d = dynamic_cast<const manipulation_metrics_data*>(data);

  /* interval averages */
  line += csv_entry_intavg(d->interval[block_manip_events::ekFREE_PICKUP].events);
  line += csv_entry_intavg(d->interval[block_manip_events::ekFREE_DROP].events);

  line +=
      csv_entry_domavg(d->interval[block_manip_events::ekFREE_PICKUP].penalties,
                       d->interval[block_manip_events::ekFREE_PICKUP].events);
  line += csv_entry_domavg(d->interval[block_manip_events::ekFREE_DROP].penalties,
                           d->interval[block_manip_events::ekFREE_DROP].events);

  line +=
      csv_entry_intavg(d->interval[block_manip_events::ekCACHE_PICKUP].events);
  line += csv_entry_intavg(d->interval[block_manip_events::ekCACHE_DROP].events);

  line +=
      csv_entry_domavg(d->interval[block_manip_events::ekCACHE_PICKUP].penalties,
                       d->interval[block_manip_events::ekCACHE_PICKUP].events);
  line +=
      csv_entry_domavg(d->interval[block_manip_events::ekCACHE_DROP].penalties,
                       d->interval[block_manip_events::ekCACHE_DROP].events);

  /* cumulative averages */
  line += csv_entry_tsavg(d->cum[block_manip_events::ekFREE_PICKUP].events, t);
  line += csv_entry_tsavg(d->cum[block_manip_events::ekFREE_DROP].events, t);

  line += csv_entry_domavg(d->cum[block_manip_events::ekFREE_PICKUP].penalties,
                           d->cum[block_manip_events::ekFREE_PICKUP].events);
  line += csv_entry_domavg(d->cum[block_manip_events::ekFREE_DROP].penalties,
                           d->cum[block_manip_events::ekFREE_DROP].events);

  line += csv_entry_tsavg(d->cum[block_manip_events::ekCACHE_PICKUP].events, t);
  line += csv_entry_tsavg(d->cum[block_manip_events::ekCACHE_DROP].events, t);

  line += csv_entry_domavg(d->cum[block_manip_events::ekCACHE_PICKUP].penalties,
                           d->cum[block_manip_events::ekCACHE_PICKUP].events);
  line += csv_entry_domavg(d->cum[block_manip_events::ekCACHE_DROP].penalties,
                           d->cum[block_manip_events::ekCACHE_DROP].events,
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(blocks, metrics, fordyca);
