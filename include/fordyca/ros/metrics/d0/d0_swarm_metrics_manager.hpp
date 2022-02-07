/**
 * \file d0_swarm_metrics_manager.hpp
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

#ifndef INCLUDE_FORDYCA_ROS_METRICS_D0_D0_SWARM_METRICS_MANAGER_HPP_
#define INCLUDE_FORDYCA_ROS_METRICS_D0_D0_SWARM_METRICS_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "cosm/ros/metrics/swarm_metrics_manager.hpp"

#include "fordyca/metrics/blocks/manipulation_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, metrics, d0);
namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class d0_swarm_metrics_manager
 * \ingroup ros metrics d0
 *
 * \brief Collects metrics from robots via ROS topics and writes the aggregated
 * result to the filesystem for process. Runs on the ROS master. Currently
 * includes:
 */

class d0_swarm_metrics_manager : public crmetrics::swarm_metrics_manager,
                                 public rer::client<d0_swarm_metrics_manager> {
 public:
  explicit d0_swarm_metrics_manager(const rmconfig::metrics_config* mconfig,
                                    const fs::path& root,
                                    size_t n_robots);

 private:
  void register_standard(const rmconfig::metrics_config* const mconfig,
                         size_t n_robots);
  void collect(const boost::shared_ptr<const fmetrics::blocks::manipulation_metrics_data>& in);

  /* clang-format off */
  std::vector<::ros::Subscriber> m_subs{};
  /* clang-format on */
};

NS_END(d0, metrics, ros, fordyca);

#endif /* INCLUDE_FORDYCA_ROS_METRICS_D0_D0_SWARM_METRICS_MANAGER_HPP_ */
