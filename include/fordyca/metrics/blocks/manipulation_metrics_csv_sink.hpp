/**
 * \file manipulation_metrics_csv_sink.hpp
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
#include "fordyca/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);
class manipulation_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class manipulation_metrics_csv_sink
 * \ingroup metrics blocks
 *
 * \brief Sink for \ref manipulation_metrics and \ref
 * manipulation_metrics_collector to output metrics to .csv.
 */
class manipulation_metrics_csv_sink final
    : public rer::client<manipulation_metrics_csv_sink>,
      public rmetrics::csv_sink {
 public:
  using collector_type = manipulation_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  manipulation_metrics_csv_sink(fs::path fpath_no_ext,
                            const rmetrics::output_mode& mode,
                            const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_data* data,
      const rtypes::timestep& t) override;
};

NS_END(blocks, metrics, fordyca);

