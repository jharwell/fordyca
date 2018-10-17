/**
 * @file stateful_tasking_initializer.cpp
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
#include "fordyca/controller/depth0/stateful_tasking_initializer.hpp"
#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/base_sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/ds/perceived_arena_map.hpp"
#include "fordyca/fsm/depth0/stateful_fsm.hpp"
#include "fordyca/params/depth0/stateful_controller_repository.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"

#include "rcppsw/task_allocation/bi_tdgraph.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/task_allocation_params.hpp"
#include "rcppsw/task_allocation/task_executive_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
using ds::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_tasking_initializer::stateful_tasking_initializer(
    const controller::block_selection_matrix* const sel_matrix,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : ER_CLIENT_INIT("fordyca.controller.depth0.tasking_initializer"),
      m_saa(saa),
      m_perception(perception),
      mc_sel_matrix(sel_matrix),
      m_graph(nullptr) {}

stateful_tasking_initializer::~stateful_tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_tasking_initializer::stateful_tasking_init(
    params::depth0::stateful_controller_repository* const stateful_repo) {
  auto* task_params = stateful_repo->parse_results<ta::task_allocation_params>();

  ER_ASSERT(block_sel_matrix(), "NULL block selection matrix");

  std::unique_ptr<ta::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_fsm>(mc_sel_matrix,
                                                     m_saa,
                                                     m_perception->map());
  auto generalist =
      new tasks::depth0::generalist(task_params, std::move(generalist_fsm));

  if (task_params->exec_est.seed_enabled) {
    uint min = task_params->exec_est.ranges.find("generalist")->second.get_min();
    uint max = task_params->exec_est.ranges.find("generalist")->second.get_max();
    ER_INFO("Seeding exec estimate for tasks: '%s'=[%u,%u]",
            generalist->name().c_str(),
            min,
            max);
    generalist->exec_estimate_init(min, max);
  }
  m_graph = new ta::bi_tdgraph(task_params);

  m_graph->set_root(generalist);
  generalist->set_atomic(true);
} /* tasking_init() */

std::unique_ptr<ta::bi_tdgraph_executive> stateful_tasking_initializer::
operator()(params::depth0::stateful_controller_repository* const stateful_repo) {
  stateful_tasking_init(stateful_repo);

  struct ta::task_executive_params p;
  p.update_exec_ests = true;
  p.update_interface_ests = true;
  return rcppsw::make_unique<ta::bi_tdgraph_executive>(&p, m_graph);
} /* initialize() */

NS_END(depth0, controller, fordyca);
