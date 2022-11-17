/**
 * \file d0_swarm_metrics_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "cosm/ros/metrics/swarm_metrics_manager.hpp"

#include "fordyca/ros/metrics/blocks/manipulation_metrics_msg.hpp"

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
  void collect(const boost::shared_ptr<const frmblocks::manipulation_metrics_msg>& msg);

  /* clang-format off */
  std::vector<::ros::Subscriber> m_subs{};
  /* clang-format on */
};

NS_END(d0, metrics, ros, fordyca);
