/**
 * @file robot_interaction_metrics_collector.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/metrics/robot_interaction_metrics_collector.hpp"
#include <limits>

#include "fordyca/metrics/robot_interaction_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, metrics);

namespace swarm = rcppsw::swarm;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
robot_interaction_metrics_collector::robot_interaction_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string robot_interaction_metrics_collector::csv_header_build(
    const std::string& header) {
  return base_metrics_collector::csv_header_build(header) +
      "cum_avg_degree_raw" + separator() +
      "cum_avg_degree_normed" + separator();
} /* csv_header_build() */

void robot_interaction_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

bool robot_interaction_metrics_collector::csv_line_build(std::string& line) {
  if (!((timestep() + 1) % interval() == 0)) {
    return false;
  }

  std::vector<double> tmp = m_cum_stats;
  std::for_each(tmp.begin(),
                tmp.end(),
                [&](auto& d) {
                  d /= (timestep() + 1);
                });

  line += std::to_string(m_cum.raw_degree(tmp)) + separator();
  line += std::to_string(m_cum.normed_degree(tmp)) + separator();
  return true;
} /* csv_line_build() */

void robot_interaction_metrics_collector::collect(
    const rcppsw::metrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const robot_interaction_metrics&>(metrics);

  auto neighbors = m.nearest_neighbors();

  if (0 == m_cum_stats.size()) {
    m_cum_stats = neighbors;
  } else {
    /*
     * Note that each robot's distance to closest neighbor does not
     * appear in the same place in the result array on subsequent timesteps. This
     * is OK, because we are doing a cumulative addition with the results as we
     * collect metrics, so it doesn't really matter that you are not doing
     * strictly piecewise addition.
     */
    std::transform(neighbors.begin(),
                   neighbors.end(),
                   m_cum_stats.begin(),
                   m_cum_stats.begin(),
                   std::plus<double>());
  }
} /* collect() */

NS_END(metrics, fordyca);
