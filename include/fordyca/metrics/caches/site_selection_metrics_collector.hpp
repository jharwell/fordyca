/**
 * \file site_selection_metrics_collector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_collector.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/metrics/caches/site_selection_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class site_selection_metrics_collector
 * \ingroup metrics caches
 *
 * \brief Collector for \ref site_selection_metrics.
 *
 * Metrics CANNOT be collected in parallel; concurrent updates to the gathered
 * stats are not supported.
 */
class site_selection_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit site_selection_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  site_selection_metrics_data m_data{};
  /* clang-format on */
};

NS_END(caches, metrics, fordyca);

