/**
 * \file env_dynamics_metrics_collector.hpp
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

