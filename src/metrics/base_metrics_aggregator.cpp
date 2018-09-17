/**
 * @file base_metrics_aggregator.cpp
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
#include "fordyca/metrics/base_metrics_aggregator.hpp"
#include "fordyca/params/metrics_params.hpp"

#include <experimental/filesystem>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fs = std::experimental::filesystem;
NS_START(fordyca, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_metrics_aggregator::base_metrics_aggregator(
    const struct params::metrics_params* params,
    const std::string& output_root)
    : ER_CLIENT_INIT("fordyca.metrics.base_aggregator"), collector_group() {
  m_metrics_path = output_root + "/" + params->output_dir;

  if (!fs::exists(m_metrics_path)) {
    fs::create_directories(m_metrics_path);
  } else {
    ER_WARN("Output metrics path '%s' already exists", m_metrics_path.c_str());
  }
}
/*******************************************************************************
 * Member Functions
 ******************************************************************************/

NS_END(metrics, fordyca);
