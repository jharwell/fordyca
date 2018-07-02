/**
 * @file tasking_initializer.cpp
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
#include "fordyca/controller/depth1/tasking_initializer.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/params/depth1/exec_estimates_params.hpp"
#include "fordyca/params/depth1/param_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"

#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/executive_params.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"
#include "rcppsw/task_allocation/task_decomposition_graph.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
tasking_initializer::tasking_initializer(
    std::shared_ptr<rcppsw::er::server>& server,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : stateful_tasking_initializer(server, saa, perception) {}

tasking_initializer::~tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tasking_initializer::depth1_tasking_init(
    params::depth1::param_repository* const param_repo) {
  auto* exec_params = param_repo->parse_results<ta::executive_params>();
  auto* est_params =
      param_repo->parse_results<params::depth1::exec_estimates_params>();

  std::unique_ptr<ta::taskable> collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          param_repo->parse_results<params::fsm_params>(),
          server(),
          saa_subsystem(),
          perception()->map());

  std::unique_ptr<ta::taskable> harvester_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_existing_cache_fsm>(
          param_repo->parse_results<params::fsm_params>(),
          server(),
          saa_subsystem(),
          perception()->map());

  auto collector =
      ta::make_task_graph_vertex<tasks::depth1::collector>(exec_params,
                                                           collector_fsm);

  auto harvester =
      ta::make_task_graph_vertex<tasks::depth1::harvester>(exec_params,
                                                           harvester_fsm);

  if (est_params->enabled) {
    std::static_pointer_cast<ta::polled_task>(harvester)->init_random(
        est_params->harvester_range.GetMin(),
        est_params->harvester_range.GetMax());
    std::static_pointer_cast<ta::polled_task>(collector)->init_random(
        est_params->collector_range.GetMin(),
        est_params->collector_range.GetMax());
  }

  graph()->set_children(tasks::depth0::foraging_task::kGeneralistName,
                        std::list<ta::task_graph_vertex>({collector, harvester}));
} /* depth1_tasking_init() */

std::unique_ptr<ta::polled_executive> tasking_initializer::operator()(
    params::depth1::param_repository* const param_repo) {
  stateful_tasking_init(param_repo);
  depth1_tasking_init(param_repo);

  return rcppsw::make_unique<ta::polled_executive>(server(), std::move(graph()));
} /* initialize() */

NS_END(depth1, controller, fordyca);
