/**
 * \file site_selection_metrics_csv_sink.cpp
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
#include "fordyca/metrics/caches/site_selection_metrics_csv_sink.hpp"

#include "fordyca/metrics/caches/site_selection_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
site_selection_metrics_csv_sink::site_selection_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
site_selection_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_metrics_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_n_successes",
    "int_n_fails",
    "int_nlopt_stopval",
    "int_nlopt_ftol",
    "int_nlopt_xtol",
    "int_nlopt_maxeval",
    "cum_n_successes",
    "cum_n_fails",
    "cum_nlopt_stopval",
    "cum_nlopt_ftol",
    "cum_nlopt_xtol",
    "cum_nlopt_maxeval",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
site_selection_metrics_csv_sink::csv_line_build(
    const rmetrics::base_metrics_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;
  auto* d = dynamic_cast<const site_selection_metrics_data*>(data);

  line += csv_entry_intavg(d->interval.n_successes);
  line += csv_entry_intavg(d->interval.n_fails);
  line += csv_entry_intavg(d->interval.nlopt_stopval);
  line += csv_entry_intavg(d->interval.nlopt_ftol);
  line += csv_entry_intavg(d->interval.nlopt_xtol);
  line += csv_entry_intavg(d->interval.nlopt_maxeval);

  line += csv_entry_tsavg(d->cum.n_successes, t);
  line += csv_entry_tsavg(d->cum.n_fails, t);
  line += csv_entry_tsavg(d->cum.nlopt_stopval, t);
  line += csv_entry_tsavg(d->cum.nlopt_ftol, t);
  line += csv_entry_tsavg(d->cum.nlopt_xtol, t);
  line += csv_entry_tsavg(d->cum.nlopt_maxeval, t, true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(caches, metrics, fordyca);
