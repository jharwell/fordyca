/**
 * \file dpo_metrics_csv_sink.cpp
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
#include "fordyca/metrics/perception/dpo_metrics_csv_sink.hpp"

#include "fordyca/metrics/perception/dpo_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_metrics_csv_sink::dpo_metrics_csv_sink(fs::path fpath_no_ext,
                                           const rmetrics::output_mode& mode,
                                           const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
dpo_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
      "int_avg_known_blocks",
      "cum_avg_known_blocks",
      "int_avg_known_caches",
      "cum_avg_known_caches",
      "int_avg_block_pheromone_density",
      "cum_avg_block_pheromone_density",
      "int_avg_cache_pheromone_density",
      "cum_avg_cache_pheromone_density"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
dpo_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                     const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;

  auto* d = dynamic_cast<const dpo_metrics_data*>(data);

  line += csv_entry_domavg(d->interval.known_blocks, d->interval.robot_count);
  line += csv_entry_domavg(d->cum.known_blocks, d->cum.robot_count);
  line += csv_entry_domavg(d->interval.known_caches, d->interval.robot_count);
  line += csv_entry_domavg(d->cum.known_caches, d->cum.robot_count);

  line +=
      csv_entry_domavg(d->interval.block_density_sum, d->interval.robot_count);
  line += csv_entry_domavg(d->cum.block_density_sum, d->cum.robot_count);
  line +=
      csv_entry_domavg(d->interval.cache_density_sum, d->interval.robot_count);
  line += csv_entry_domavg(d->cum.cache_density_sum, d->cum.robot_count, true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(perception, metrics, fordyca);
