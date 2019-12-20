/**
 * \file depth2/task_executive_builder.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_TASK_EXECUTIVE_BUILDER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_TASK_EXECUTIVE_BUILDER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <memory>

#include "fordyca/controller/depth1/task_executive_builder.hpp"

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
 * \class task_executive_builder
 * \ingroup controller depth2
 *
 * \brief A helper class to offload initialization of the task tree and
 * executive for depth2 foraging.
 */
class task_executive_builder : public depth1::task_executive_builder,
                            public rer::client<task_executive_builder> {
 public:
  task_executive_builder(const controller::block_sel_matrix* bsel_matrix,
                      const controller::cache_sel_matrix* csel_matrix,
                      crfootbot::footbot_saa_subsystem* saa,
                      base_perception_subsystem* perception) RCSW_COLD;
  ~task_executive_builder(void) override RCSW_COLD;

  std::unique_ptr<cta::bi_tdgraph_executive>
  operator()(const config::depth2::controller_repository& config_repo,
             rmath::rng* rng) RCSW_COLD;

  using depth1::task_executive_builder::tasking_map;

 protected:
  tasking_map depth2_tasks_create(
      const config::depth2::controller_repository& config_repo,
      cta::ds::bi_tdgraph* graph,
      rmath::rng* rng) RCSW_COLD;

  void depth2_exec_est_init(const config::depth2::controller_repository& config_repo,
                            const tasking_map& map,
                            cta::ds::bi_tdgraph* graph,
                            rmath::rng* rng) RCSW_COLD;
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_TASK_EXECUTIVE_BUILDER_HPP_ */
