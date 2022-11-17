/**
 * \file dpo_metrics_csv_sink.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
NS_START(fordyca, metrics, perception);
class dpo_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dpo_metrics_csv_sink
 * \ingroup metrics blocks
 *
 * \brief Sink for \ref dpo_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported.
 */
class dpo_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = dpo_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  dpo_metrics_csv_sink(fs::path fpath_no_ext,
                       const rmetrics::output_mode& mode,
                       const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_data* data,
      const rtypes::timestep& t) override;
};

NS_END(perception, metrics, fordyca);

