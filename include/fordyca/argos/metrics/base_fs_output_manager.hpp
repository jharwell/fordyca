/**
 * \file base_fs_output_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ds/config/grid2D_config.hpp"
#include "cosm/argos/metrics/fs_output_manager.hpp"

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
                               public cargos::metrics::fs_output_manager {
 public:
  base_fs_output_manager(const rmconfig::metrics_config* mconfig,
                          const cdconfig::grid2D_config* gconfig,
                          const fs::path& output_root,
                          size_t n_block_clusters);
  ~base_fs_output_manager(void) override = default;

  void collect_from_sm(const fasupport::argos_swarm_manager* sm);
};

NS_END(metrics, argos, fordyca);

