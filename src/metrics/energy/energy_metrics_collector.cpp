/**
 * @file energy_metrics_collector.cpp
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
 /*******************************************************************************
  * Includes
  ******************************************************************************/
 #include "fordyca/metrics/energy/energy_metrics_collector.hpp"
 #include "fordyca/metrics/energy/energy_opt_metrics.hpp"

 /*******************************************************************************
  * Namespaces
  ******************************************************************************/
 NS_START(fordyca, metrics, energy);

 /*******************************************************************************
  * Constructors/Destructor
  ******************************************************************************/
 energy_metrics_collector::energy_metrics_collector(const std::string& ofname,
                                                    uint interval)
     : base_metrics_collector(ofname, interval) {}

 /*******************************************************************************
  * Member Functions
  ******************************************************************************/
  std::string energy_metrics_collector::csv_header_build(
      const std::string& header) {
    // clang-format off
    return base_metrics_collector::csv_header_build(header) +
        "n_robots" + separator() +
        "avg_energy_level" + separator() +
        "avg_robots_at_nest" + separator();
    // clang-format on
  } /* csv_header_build() */

  void energy_metrics_collector::reset(void) {
    base_metrics_collector::reset();
    reset_after_interval();
  } /* reset() */

  bool energy_metrics_collector::csv_line_build(std::string& line) {
    if (!((timestep() + 1) % interval() == 0)) {
      return false;
    }
    line += std::to_string(m_stats.cum_robots) + separator();
    if (m_stats.cum_robots > 0) {
      line += std::to_string(m_stats.cum_robots_at_nest /
                             static_cast<double>(m_stats.cum_robots)) +
              separator();
      line += std::to_string(m_stats.cum_energy /
                             static_cast<double>(m_stats.cum_robots)) +
              separator();
    } else {
      line += "0" + separator() + "0" + separator();
    }
    return true;
  } /* csv_line_build() */

  void energy_metrics_collector::collect(
      const rcppsw::metrics::base_metrics& metrics) {
    auto& m = dynamic_cast<const energy_metrics&>(metrics);
    ++m_stats.cum_robots;
    m_stats.cum_energy += m.energy_level();
    m_stats.cum_robots_at_nest += m.is_charging();
  } /* collect() */

  void energy_metrics_collector::reset_after_interval(void) {
    m_stats = {0, 0, 0};
  } /* reset_after_interval() */

  NS_END(energy, metrics, fordyca);
