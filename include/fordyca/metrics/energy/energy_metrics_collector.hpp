/**
 * @file energy_metrics_collector.hpp
 *
 * @copyright 2018 Anthony Chen/John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_METRICS_ENERGY_METRICS_COLLECTOR_HPP_
#define INCLUDE_FORDYCA_METRICS_ENERGY_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/patterns/visitor/visitable.hpp"
#include "rcppsw/metrics/base_metrics_collector.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics, energy);
namespace visitor = rcppsw::patterns::visitor;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class energy_metrics_collector
 * @ingroup metrics energy
 *
 * @brief Collector for \ref energy_metrics.
 *
 * Metrics are written out at the specified collection interval.
 */
class energy_metrics_collector : public rcppsw::metrics::base_metrics_collector,
                                    public visitor::visitable_any<transport_metrics_collector> {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   */
  energy_metrics_collector(const std::string& ofname, uint interval);

  void reset(void) override;
  void collect(const rcppsw::metrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  struct stats {

    /**
     * @brief Total robots
     */
    uint cum_robots;

    /**
     * @brief Amount of energy charged
     */
    double cum_energy;

    /**
     * @brief Total # robots entered nest
     */
    uint cum_robots_at_nest;

  };

  std::string csv_header_build(const std::string& header) override;
  bool csv_line_build(std::string& line) override;

  struct stats m_stats;
};

NS_END(energy, metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_ENERGY_METRICS_COLLECTOR_HPP_ */
