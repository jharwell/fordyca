/**
 * \file lifecycle_metrics_csv_sink.cpp
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
#include "fordyca/metrics/caches/lifecycle_metrics_csv_sink.hpp"

#include "fordyca/metrics/caches/lifecycle_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
lifecycle_metrics_csv_sink::lifecycle_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> lifecycle_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_metrics_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_created",
    "int_depleted",
    "int_discarded",

    "int_avg_created",
    "int_avg_depleted",
    "int_avg_discarded",
    "int_avg_depletion_age",

    "cum_avg_created",
    "cum_avg_depleted",
    "cum_avg_discarded",
    "cum_avg_depletion_age"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> lifecycle_metrics_csv_sink::csv_line_build(
    const rmetrics::base_metrics_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;
  auto* d = dynamic_cast<const lifecycle_metrics_data*>(data);

  /* raw metrics */
  line += rcppsw::to_string(d->interval.created) + separator();
  line += rcppsw::to_string(d->interval.depleted) + separator();
  line += rcppsw::to_string(d->interval.discarded) + separator();

  /* interval averages */
  line += csv_entry_intavg(d->interval.created);
  line += csv_entry_intavg(d->interval.depleted);
  line += csv_entry_intavg(d->interval.discarded);
  line += csv_entry_domavg(d->interval.depletion_sum.v(), d->interval.depleted);

  /* cumulative averages */
  line += csv_entry_tsavg(d->cum.created, t);
  line += csv_entry_tsavg(d->cum.depleted, t);
  line += csv_entry_tsavg(d->cum.discarded, t);

  line += csv_entry_domavg(d->cum.depletion_sum.v(),
                       d->cum.depleted,
                       true);
  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(caches, metrics, fordyca);
