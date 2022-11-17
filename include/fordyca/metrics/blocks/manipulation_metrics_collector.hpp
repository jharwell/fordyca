/**
 * \file manipulation_metrics_collector.hpp
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

#include "fordyca/metrics/blocks/manipulation_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, blocks);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class manipulation_metrics_collector
 * \ingroup metrics blocks
 *
 * \brief Collector for \ref manipulation_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the specified
 * collection interval.
 */
class manipulation_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit manipulation_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

#if defined(COSM_PAL_TARGET_ROS)
  void collect(const manipulation_metrics_data& data) { m_data += data; }
#endif

 private:
  /* clang-format off */
  manipulation_metrics_data m_data{};
  /* clang-format on */
};

NS_END(blocks, metrics, fordyca);
