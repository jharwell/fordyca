/**
 * @file task_executive_builder.cpp
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
#include "fordyca/controller/depth1/task_executive_builder.hpp"

#include <vector>

#include "rcppsw/ta/bi_tdgraph_executive.hpp"
#include "rcppsw/ta/config/task_alloc_config.hpp"
#include "rcppsw/ta/config/task_executive_config.hpp"
#include "rcppsw/ta/ds/bi_tdgraph.hpp"
#include "rcppsw/ta/bi_tdgraph_allocator.hpp"

#include "fordyca/config/depth1/controller_repository.hpp"
#include "fordyca/config/exploration_config.hpp"
#include "fordyca/controller/base_perception_subsystem.hpp"
#include "fordyca/controller/dpo_perception_subsystem.hpp"
#include "fordyca/controller/mdpo_perception_subsystem.hpp"
#include "fordyca/ds/dpo_semantic_map.hpp"
#include "fordyca/fsm/depth0/dpo_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/expstrat/block_factory.hpp"
#include "fordyca/fsm/expstrat/cache_factory.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"

#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"
#include "cosm/robots/footbot/footbot_sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
task_executive_builder::task_executive_builder(
    const controller::block_sel_matrix* bsel_matrix,
    const controller::cache_sel_matrix* csel_matrix,
    crfootbot::footbot_saa_subsystem* const saa,
    base_perception_subsystem* const perception)
    : ER_CLIENT_INIT("fordyca.controller.depth1.task_executive_builder"),
      mc_csel_matrix(csel_matrix),
      mc_bsel_matrix(bsel_matrix),
      m_saa(saa),
      m_perception(perception) {}

task_executive_builder::~task_executive_builder(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
task_executive_builder::tasking_map task_executive_builder::depth1_tasks_create(
    const config::depth1::controller_repository& config_repo,
    rta::ds::bi_tdgraph* const graph,
    rmath::rng* rng) {
  auto* task_config = config_repo.config_get<rta::config::task_alloc_config>();
  auto* exp_config = config_repo.config_get<config::exploration_config>();
  fsm::expstrat::block_factory block_factory;
  fsm::expstrat::cache_factory cache_factory;
  fsm::expstrat::foraging_expstrat::params expbp(
      saa(), nullptr, mc_csel_matrix, m_perception->dpo_store());

  ER_ASSERT(nullptr != mc_bsel_matrix, "NULL block selection matrix");
  ER_ASSERT(nullptr != mc_csel_matrix, "NULL cache selection matrix");

  fsm::fsm_ro_params params = {
    .bsel_matrix = block_sel_matrix(),
    .csel_matrix = mc_csel_matrix,
    .store = m_perception->dpo_store(),
    .exp_config = *exp_config
  };

  auto generalist_fsm = std::make_unique<fsm::depth0::free_block_to_nest_fsm>(
      &params,
      saa(),
      block_factory.create(exp_config->block_strategy, &expbp, rng),
      rng);
  auto collector_fsm = std::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
      &params,
      saa(),
      cache_factory.create(exp_config->cache_strategy, &expbp, rng),
      rng);

  auto harvester_fsm =
      std::make_unique<fsm::depth1::block_to_existing_cache_fsm>(&params,
                                                                 saa(),
                                                                 rng);

  auto collector =
      std::make_unique<tasks::depth1::collector>(task_config,
                                                 std::move(collector_fsm));

  auto harvester =
      std::make_unique<tasks::depth1::harvester>(task_config,
                                                 std::move(harvester_fsm));
  auto generalist =
      std::make_unique<tasks::depth0::generalist>(task_config,
                                                  std::move(generalist_fsm));
  generalist->set_partitionable(true);
  generalist->set_atomic(false);

  graph->set_root(std::move(generalist));
  rta::ds::bi_tdgraph::vertex_vector children;
  children.push_back(std::move(harvester));
  children.push_back(std::move(collector));
  graph->install_tab(tasks::depth0::foraging_task::kGeneralistName,
                     std::move(children), rng);
  return tasking_map{
      {"generalist",
       graph->find_vertex(tasks::depth0::foraging_task::kGeneralistName)},
      {"collector",
       graph->find_vertex(tasks::depth1::foraging_task::kCollectorName)},
      {"harvester",
       graph->find_vertex(tasks::depth1::foraging_task::kHarvesterName)}};
} /* depth1_tasks_create() */

void task_executive_builder::depth1_exec_est_init(
    const config::depth1::controller_repository& config_repo,
    const tasking_map& map,
    rta::ds::bi_tdgraph* const graph,
    rmath::rng* rng) {
  auto* task_config = config_repo.config_get<rta::config::task_alloc_config>();
  if (!task_config->exec_est.seed_enabled) {
    return;
  }
  /*
   * Generalist is not partitionable in depth 0 initialization, so this has
   * not been done.
   */
  auto harvester = map.find("harvester")->second;
  auto collector = map.find("collector")->second;
  auto generalist = map.find("generalist")->second;

  if (0 == rng->uniform(0, 1)) {
    graph->root_tab()->last_subtask(harvester);
  } else {
    graph->root_tab()->last_subtask(collector);
  }

  rmath::rangeu g_bounds =
      task_config->exec_est.ranges.find("generalist")->second;

  rmath::rangeu h_bounds =
      task_config->exec_est.ranges.find("harvester")->second;
  rmath::rangeu c_bounds =
      task_config->exec_est.ranges.find("collector")->second;
  ER_INFO("Seeding exec estimate for tasks: '%s'=%s '%s'=%s '%s'=%s",
          generalist->name().c_str(),
          g_bounds.to_str().c_str(),
          harvester->name().c_str(),
          h_bounds.to_str().c_str(),
          collector->name().c_str(),
          c_bounds.to_str().c_str());
  generalist->exec_estimate_init(g_bounds, rng);
  harvester->exec_estimate_init(h_bounds, rng);
  collector->exec_estimate_init(c_bounds, rng);
} /* depth1_exec_est_init() */

std::unique_ptr<rta::bi_tdgraph_executive> task_executive_builder::operator()(
    const config::depth1::controller_repository& config_repo,
    rmath::rng* rng) {
  auto* task_config = config_repo.config_get<rta::config::task_alloc_config>();
  auto variant =
      std::make_unique<rta::ds::ds_variant>(rta::ds::bi_tdgraph(task_config));
  auto graph = boost::get<rta::ds::bi_tdgraph>(variant.get());
  auto map = depth1_tasks_create(config_repo, graph, rng);
  const auto* execp =
      config_repo.config_get<rta::config::task_executive_config>();
  const auto* allocp =
      config_repo.config_get<rta::config::task_alloc_config>();

  /* can be omitted if the user wants the default values */
  if (nullptr == execp) {
    execp = std::make_unique<rta::config::task_executive_config>().get();
  }

  /*
   * Only necessary if we are using the stochastic greedy neighborhood policy;
   * causes segfaults due to asserts otherwise.
   */
  if (rta::bi_tdgraph_allocator::kPolicyStochGreedyNBHD == allocp->policy) {
    graph->active_tab_init(allocp->stoch_greedy_nbhd.tab_init_policy, rng);
  }
  depth1_exec_est_init(config_repo, map, graph, rng);

  return std::make_unique<rta::bi_tdgraph_executive>(execp,
                                                     allocp,
                                                     std::move(variant),
                                                     rng);
} /* initialize() */

NS_END(depth1, controller, fordyca);
