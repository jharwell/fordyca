/**
 * \file env_dynamics_metrics_csv_sink.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_CSV_SINK_HPP_
#define INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_CSV_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/csv_sink.hpp"

#include "fordyca/metrics/tv/env_dynamics_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tv);
class env_dynamics_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_metrics_csv_sink
 * \ingroup metrics tv
 *
 * \brief Sink for \ref env_dynamics_pmetrics.
 *
 * Metrics CANNOT be collected in parallel; concurrent updates to the gathered
 * stats are not supported.
 */
class env_dynamics_metrics_csv_sink final
    : public rmetrics::csv_sink {
 public:
  using collector_type = env_dynamics_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  env_dynamics_metrics_csv_sink(fs::path fpath_no_ext,
                                const rmetrics::output_mode& mode,
                                const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_metrics_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_metrics_data* data,
      const rtypes::timestep& t) override;
};

NS_END(tv, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_CSV_SINK_HPP_ */
