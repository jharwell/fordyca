/**
 * @file metrics_config.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONFIG_METRICS_CONFIG_HPP_
#define INCLUDE_FORDYCA_CONFIG_METRICS_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/config/grid_config.hpp"
#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct metrics_config
 * @ingroup fordyca config
 */
struct metrics_config : public rconfig::base_config {
  std::string fsm_movement_fname{};
  std::string fsm_collision_counts_fname{};
  std::string fsm_collision_locs_fname{};

  std::string block_acq_counts_fname{};
  std::string block_acq_locs_fname{};
  std::string block_acq_explore_locs_fname{};
  std::string block_acq_vector_locs_fname{};
  std::string block_transport_fname{};
  std::string block_manipulation_fname{};

  std::string cache_acq_counts_fname{};
  std::string cache_acq_locs_fname{};
  std::string cache_acq_explore_locs_fname{};
  std::string cache_acq_vector_locs_fname{};
  std::string cache_utilization_fname{};
  std::string cache_lifecycle_fname{};
  std::string cache_locations_fname{};

  std::string task_execution_generalist_fname{};
  std::string task_execution_collector_fname{};
  std::string task_execution_harvester_fname{};
  std::string task_execution_cache_starter_fname{};
  std::string task_execution_cache_finisher_fname{};
  std::string task_execution_cache_transferer_fname{};
  std::string task_execution_cache_collector_fname{};

  std::string task_tab_generalist_fname{};
  std::string task_tab_harvester_fname{};
  std::string task_tab_collector_fname{};

  std::string task_distribution_fname{};

  std::string output_dir{};

  std::string swarm_dist_pos2D_fname{};
  std::string swarm_convergence_fname{};
  std::string temporal_variance_fname{};
  std::string perception_mdpo_fname{};
  std::string perception_dpo_fname{};

  grid_config arena_grid{};
  uint collect_interval{0};
};

NS_END(config, fordyca);

#endif /* INCLUDE_FORDYCA_CONFIG_METRICS_CONFIG_HPP_ */
