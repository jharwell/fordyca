/**
 * \file fordyca_metrics_manager.hpp
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

#ifndef INCLUDE_FORDYCA_METRICS_FORDYCA_METRICS_MANAGER_HPP_
#define INCLUDE_FORDYCA_METRICS_FORDYCA_METRICS_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ds/config/grid2D_config.hpp"
#include "cosm/metrics/cosm_metrics_manager.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace support {
class base_loop_functions;
}
namespace controller {
class foraging_controller;
} /* namespace controller */

NS_START(metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class fordyca_metrics_manager
 * \ingroup metrics
 *
 * \brief Extends the \ref cmetrics::cosm_metrics_manager for the FORDYCA
 * project.
 */
class fordyca_metrics_manager : public rer::client<fordyca_metrics_manager>,
                                public cmetrics::cosm_metrics_manager {
 public:
  fordyca_metrics_manager(const rmconfig::metrics_config* mconfig,
                          const cdconfig::grid2D_config* gconfig,
                          const fs::path& output_root,
                          size_t n_block_clusters);
  ~fordyca_metrics_manager(void) override = default;

  void collect_from_loop(const support::base_loop_functions* loop);
};

NS_END(metrics, fordyca);

#endif /* INCLUDE_FORDYCA_METRICS_FORDYCA_METRICS_MANAGER_HPP_ */
