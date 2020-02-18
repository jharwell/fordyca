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

#ifndef INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/base_metrics_collector.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class env_dynamics_metrics_collector
 * \ingroup metrics blocks
 *
 * \brief Collector for \ref env_dynamics_metrics.
 *
 * Metrics CANNOT be collected in parallel; concurrent updates to the gathered
 * stats are not supported. Metrics are written out every timestep.
 */
class env_dynamics_metrics_collector final
    : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname The output file name.
   */
  explicit env_dynamics_metrics_collector(const std::string& ofname);

  void collect(const rmetrics::base_metrics& metrics) override;

 private:
  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  /* clang-format off */
  double           m_avg_motion_throttle{0.0};
  rtypes::timestep m_block_manip_penalty{0};
  rtypes::timestep m_cache_usage_penalty{0};
  /* clang-format on */
};

NS_END(tv, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_TV_ENV_DYNAMICS_METRICS_COLLECTOR_HPP_ */
