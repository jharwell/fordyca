/**
 * \file env_dynamics_metrics_collector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/metrics/base_collector.hpp"

#include "fordyca/metrics/tv/env_dynamics_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_metrics_collector
 * \ingroup metrics tv
 *
 * \brief Collector for \ref env_dynamics_metrics.
 *
 * Metrics CANNOT be collected in parallel; concurrent updates to the gathered
 * stats are not supported.
 */
class env_dynamics_metrics_collector final
    : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit env_dynamics_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }


 private:
  /* clang-format off */
  env_dynamics_metrics_data m_data{};
  /* clang-format on */
};

NS_END(tv, metrics, fordyca);

