/**
 * @file metrics_params.hpp
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

#ifndef INCLUDE_FORDYCA_PARAMS_METRICS_PARAMS_HPP_
#define INCLUDE_FORDYCA_PARAMS_METRICS_PARAMS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/params/base_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct metrics_params
 * @ingroup params
 */
struct metrics_params : public rcppsw::params::base_params {
  std::string collision_fname{""};

  std::string block_acquisition_fname{""};
  std::string block_transport_fname{""};
  std::string block_manipulation_fname{""};

  std::string cache_acquisition_fname{""};
  std::string cache_utilization_fname{""};
  std::string cache_lifecycle_fname{""};

  std::string task_execution_generalist_fname{""};
  std::string task_execution_collector_fname{""};
  std::string task_execution_harvester_fname{""};
  std::string task_generalist_tab_fname{""};

  std::string distance_fname{""};
  std::string output_dir{""};

  std::string perception_world_model_fname{""};

  uint collect_interval{0};
};

NS_END(params, fordyca);

#endif /* INCLUDE_FORDYCA_PARAMS_METRICS_PARAMS_HPP_ */
