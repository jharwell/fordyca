/**
 * \file d0_metrics_manager.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_D0_D0_METRICS_MANAGER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D0_D0_METRICS_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/metrics/fordyca_metrics_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, d0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class d0_metrics_manager
 * \ingroup support d0
 *
 * \brief Aggregates and metrics metric collection for d0 foraging. That
 * includes:
 *
 * - FSM distance/block acquisition metrics
 */

class d0_metrics_manager : public metrics::fordyca_metrics_manager,
                                  public rer::client<d0_metrics_manager> {
 public:
  d0_metrics_manager(const rmconfig::metrics_config* mconfig,
                     const cdconfig::grid2D_config* gconfig,
                     const fs::path& output_root,
                     size_t n_block_clusters);

  template<class T>
  void collect_from_controller(const T* controller);
};

NS_END(d0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D0_D0_METRICS_MANAGER_HPP_ */
