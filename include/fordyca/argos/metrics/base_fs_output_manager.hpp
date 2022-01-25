/**
 * \file base_fs_output_manager.hpp
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

#ifndef INCLUDE_FORDYCA_ARGOS_METRICS_BASE_FS_OUTPUT_MANAGER_HPP_
#define INCLUDE_FORDYCA_ARGOS_METRICS_BASE_FS_OUTPUT_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ds/config/grid2D_config.hpp"
#include "cosm/pal/metrics/fs_output_manager.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::argos::support {
class argos_swarm_manager;
}
namespace fordyca::controller {
class foraging_controller;
} /* namespace controller */

NS_START(fordyca, argos, metrics);

namespace fs = std::filesystem;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_fs_output_manager
 * \ingroup argos metrics
 *
 * \brief Extends \ref cpargos::base_fs_output_manager for the FORDYCA project,
 * when building for ARGoS.
 */
class base_fs_output_manager : public rer::client<base_fs_output_manager>,
                               public cpmetrics::fs_output_manager {
 public:
  base_fs_output_manager(const rmconfig::metrics_config* mconfig,
                          const cdconfig::grid2D_config* gconfig,
                          const fs::path& output_root,
                          size_t n_block_clusters);
  ~base_fs_output_manager(void) override = default;

  void collect_from_sm(const fasupport::argos_swarm_manager* sm);
};

NS_END(metrics, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_METRICS_BASE_FS_OUTPUT_MANAGER_HPP_ */
