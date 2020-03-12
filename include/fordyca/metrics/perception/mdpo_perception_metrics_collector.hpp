/**
 * \file mdpo_perception_metrics_collector.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_PERCEPTION_MDPO_PERCEPTION_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_PERCEPTION_MDPO_PERCEPTION_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <atomic>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "fordyca/fordyca.hpp"
#include "cosm/fsm/cell2D_states.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, perception);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class mdpo_perception_metrics_collector
 * \ingroup metrics blocks
 *
 * \brief Collector for \ref mdpo_perception_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the specified
 * collection interval.
 */
class mdpo_perception_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname_stem The output file name stem.
   * \param interval Collection interval.
   */
  mdpo_perception_metrics_collector(const std::string& ofname_stem,
                                    const rtypes::timestep& interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  struct stats {
    std::atomic_uint    states[cfsm::cell2D_states::ekST_MAX_STATES];
    std::atomic<double> known_percent{0.0};
    std::atomic<double> unknown_percent{0.0};
    std::atomic_uint    robots{0};
  };

  /* clang-format off */
  struct stats m_interval{};
  struct stats m_cum{};
  /* clang-format on */
};

NS_END(perception, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_PERCEPTION_MDPO_PERCEPTION_METRICS_COLLECTOR_HPP_ */
