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
#include <vector>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/params/depth1/exec_estimates_params.hpp"
#include "fordyca/params/depth1/param_repository.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"

#include "rcppsw/task_allocation/bifurcating_tdgraph.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/executive_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
tasking_initializer::tasking_initializer(
    const controller::block_selection_matrix* bsel_matrix,
    const controller::cache_selection_matrix* csel_matrix,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : stateful_tasking_initializer(bsel_matrix, saa, perception),
      ER_CLIENT_INIT("fordyca.controller.depth1.tasking_initializer"),
      mc_sel_matrix(csel_matrix) {}

tasking_initializer::~tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tasking_initializer::depth1_tasking_init(
    params::depth1::param_repository* const param_repo) {
  auto* exec_params = param_repo->parse_results<ta::executive_params>();
  auto* est_params =
      param_repo->parse_results<params::depth1::exec_estimates_params>();
  ER_ASSERT(block_sel_matrix(), "NULL block selection matrix");
  ER_ASSERT(cache_sel_matrix(), "NULL cache selection matrix");
  std::unique_ptr<ta::taskable> collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          cache_sel_matrix(), saa_subsystem(), perception()->map());

  std::unique_ptr<ta::taskable> harvester_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_existing_cache_fsm>(
          block_sel_matrix(),
          mc_sel_matrix,
          saa_subsystem(),
          perception()->map());

  tasks::depth1::collector* collector =
      new tasks::depth1::collector(exec_params, std::move(collector_fsm));

  auto harvester =
      new tasks::depth1::harvester(exec_params, std::move(harvester_fsm));

  if (est_params->enabled) {
    static_cast<ta::polled_task*>(harvester)->init_random(
        est_params->harvester_range.GetMin(),
        est_params->harvester_range.GetMax());
    static_cast<ta::polled_task*>(collector)->init_random(
        est_params->collector_range.GetMin(),
        est_params->collector_range.GetMax());
    /*
     * Generalist is not partitionable in depth 0 initialization, so this has
     * not been done.
     */
    if (0 == std::rand() % 2) {
      static_cast<ta::partitionable_polled_task*>(graph()->root())
          ->init_random(collector,
                        est_params->generalist_range.GetMin(),
                        est_params->generalist_range.GetMax());
    } else {
      static_cast<ta::partitionable_polled_task*>(graph()->root())
          ->init_random(harvester,
                        est_params->generalist_range.GetMin(),
                        est_params->generalist_range.GetMax());
    }
  }
  graph()->root()->set_partitionable(true);
  graph()->root()->set_atomic(false);
  graph()->set_children(tasks::depth0::foraging_task::kGeneralistName,
                        std::vector<ta::polled_task*>({harvester, collector}));
} /* depth1_tasking_init() */

std::unique_ptr<ta::bifurcating_tdgraph_executive> tasking_initializer::operator()(
    params::depth1::param_repository* const param_repo) {
  stateful_tasking_init(param_repo);

  depth1_tasking_init(param_repo);

  return rcppsw::make_unique<ta::bifurcating_tdgraph_executive>(graph());
} /* initialize() */

NS_END(depth1, controller, fordyca);
