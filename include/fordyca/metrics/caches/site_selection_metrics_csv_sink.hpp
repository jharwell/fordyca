/**
 * \file site_selection_metrics_csv_sink.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/csv_sink.hpp"
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);
class site_selection_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class site_selection_metrics_csv_sink
 * \ingroup metrics caches
 *
 * \brief Sink for \ref site_selection_metrics and \ref
 * site_selection_metrics_collector to output metrics to .csv.
 */
class site_selection_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = site_selection_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  site_selection_metrics_csv_sink(fs::path fpath_no_ext,
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

