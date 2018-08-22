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
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/params/depth0/exec_estimates_params.hpp"
#include "fordyca/params/depth0/stateful_param_repository.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"

#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph_executive.hpp"
#include "rcppsw/task_allocation/executive_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth0);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateful_tasking_initializer::stateful_tasking_initializer(
    std::shared_ptr<rcppsw::er::server> server,
    const controller::block_selection_matrix* const sel_matrix,
    controller::saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : client(server),
      m_saa(saa),
      m_perception(perception),
      mc_sel_matrix(sel_matrix),
      m_graph(nullptr) {}

stateful_tasking_initializer::~stateful_tasking_initializer(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stateful_tasking_initializer::stateful_tasking_init(
    params::depth0::stateful_param_repository* const stateful_repo) {
  auto* exec_params = stateful_repo->parse_results<ta::executive_params>();
  auto* est_params =
      stateful_repo->parse_results<params::depth0::exec_estimates_params>();
  ER_ASSERT(block_sel_matrix(), "FATAL: NULL block selection matrix");

  std::unique_ptr<ta::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_foraging_fsm>(
          client::server_ref(), mc_sel_matrix, m_saa, m_perception->map());

  auto generalist = new tasks::depth0::generalist(exec_params,
                                                  std::move(generalist_fsm));

  if (est_params->enabled) {
    static_cast<ta::polled_task*>(generalist)
        ->init_random(est_params->generalist_range.GetMin(),
                      est_params->generalist_range.GetMax());
  }

  m_graph = new ta::bifurcating_tdgraph(client::server_ref());

  m_graph->set_root(generalist);
  generalist->set_atomic(true);
} /* tasking_init() */

std::unique_ptr<ta::bifurcating_tdgraph_executive> stateful_tasking_initializer::
operator()(params::depth0::stateful_param_repository* const stateful_repo) {
  stateful_tasking_init(stateful_repo);

  return rcppsw::make_unique<ta::bifurcating_tdgraph_executive>(
      client::server_ref(),
      m_graph);
} /* initialize() */

NS_END(depth0, controller, fordyca);
