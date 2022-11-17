/**
 * \file mdpo_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/metrics/perception/mdpo_metrics_csv_sink.hpp"

#include "fordyca/metrics/perception/mdpo_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
mdpo_metrics_csv_sink::mdpo_metrics_csv_sink(fs::path fpath_no_ext,
                                             const rmetrics::output_mode& mode,
                                             const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
mdpo_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_ST_EMPTY_inaccuracies",
    "int_avg_ST_HAS_BLOCK_inaccuracies",
    "int_avg_ST_HAS_CACHE_inaccuracies",
    "cum_avg_ST_EMPTY_inaccuracies",
    "cum_avg_ST_HAS_BLOCK_inaccuracies",
    "cum_avg_ST_HAS_CACHE_inaccuracies",
    "int_avg_known_percentage",
    "int_avg_unknown_percentage",
    "int_avg_knowledge_ratio",
    "cum_avg_known_percentage",
    "cum_avg_unknown_percentage",
    "cum_avg_knowledge_ratio"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
mdpo_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                      const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;

  auto* d = dynamic_cast<const mdpo_metrics_data*>(data);

  line += csv_entry_intavg(d->interval.states[cfsm::cell2D_state::ekST_EMPTY]);
  line +=
      csv_entry_intavg(d->interval.states[cfsm::cell2D_state::ekST_HAS_BLOCK]);
  line +=
      csv_entry_intavg(d->interval.states[cfsm::cell2D_state::ekST_HAS_CACHE]);
  line += csv_entry_tsavg(d->cum.states[cfsm::cell2D_state::ekST_EMPTY], t);
  line += csv_entry_tsavg(d->cum.states[cfsm::cell2D_state::ekST_HAS_BLOCK], t);
  line += csv_entry_tsavg(d->cum.states[cfsm::cell2D_state::ekST_HAS_CACHE], t);

  line += csv_entry_intavg(d->interval.known_percent);
  line += csv_entry_intavg(d->interval.unknown_percent);
  line +=
      csv_entry_domavg(d->interval.known_percent, d->interval.unknown_percent);
  line += csv_entry_tsavg(d->cum.known_percent, t);
  line += csv_entry_tsavg(d->cum.unknown_percent, t);
  line += csv_entry_domavg(
      d->interval.known_percent, d->interval.unknown_percent, true);
  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(perception, metrics, fordyca);
