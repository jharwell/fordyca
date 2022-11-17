/**
 * \file lifecycle_metrics_csv_sink.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/metrics/csv_sink.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);
class lifecycle_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class lifecycle_metrics_csv_sink
 * \ingroup metrics caches
 *
 * \brief Sink for \ref lifecycle_metrics and \ref
 * lifecycle_metrics_collector to output metrics to .csv.
 */
class lifecycle_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = lifecycle_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  lifecycle_metrics_csv_sink(fs::path fpath_no_ext,
                            const rmetrics::output_mode& mode,
                            const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_data* data,
      const rtypes::timestep& t) override;
};

NS_END(caches, metrics, fordyca);

