/**
 * @file depth2/tasking_initializer.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_TASKING_INITIALIZER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_TASKING_INITIALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/controller/depth1/tasking_initializer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace config {
namespace depth2 { class controller_repository; }
}

NS_START(controller, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class tasking_initializer
 * @ingroup fordyca controller depth2
 *
 * @brief A helper class to offload initialization of the task tree for depth2
 * foraging.
 */
class tasking_initializer : public depth1::tasking_initializer,
                            public rer::client<tasking_initializer> {
 public:
  tasking_initializer(const controller::block_sel_matrix* bsel_matrix,
                      const controller::cache_sel_matrix* csel_matrix,
                      crfootbot::footbot_saa_subsystem* saa,
                      base_perception_subsystem* perception) RCSW_COLD;
  ~tasking_initializer(void) override RCSW_COLD;

  std::unique_ptr<rta::bi_tdgraph_executive>
  operator()(const config::depth2::controller_repository& config_repo) RCSW_COLD;

  using depth1::tasking_initializer::tasking_map;

 protected:
  tasking_map depth2_tasks_create(
      const config::depth2::controller_repository& config_repo,
      rta::ds::bi_tdgraph* graph) RCSW_COLD;

  void depth2_exec_est_init(const config::depth2::controller_repository& config_repo,
                            const tasking_map& map,
                            rta::ds::bi_tdgraph* graph) RCSW_COLD;
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_TASKING_INITIALIZER_HPP_ */
