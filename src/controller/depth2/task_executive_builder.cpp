/**
 * \file task_executive_builder.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/depth2/task_executive_builder.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/ta/bi_tdgraph_allocator.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/config/task_alloc_config.hpp"
#include "cosm/ta/config/task_executive_config.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"
#include "cosm/arena/repr/light_type_index.hpp"

#include "fordyca/config/depth2/controller_repository.hpp"
#include "fordyca/config/exploration_config.hpp"
#include "fordyca/controller/foraging_perception_subsystem.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_new_cache_fsm.hpp"
#include "fordyca/fsm/depth2/cache_transferer_fsm.hpp"
#include "fordyca/fsm/expstrat/block_factory.hpp"
#include "fordyca/fsm/expstrat/cache_factory.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth2/cache_collector.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
task_executive_builder::task_executive_builder(
    const controller::block_sel_matrix* bsel_matrix,
    const controller::cache_sel_matrix* csel_matrix,
    crfootbot::footbot_saa_subsystem* const saa,
    foraging_perception_subsystem* const perception)
    : depth1::task_executive_builder(bsel_matrix, csel_matrix, saa, perception),
      ER_CLIENT_INIT("fordyca.controller.depth2.task_executive_builder") {}

task_executive_builder::~task_executive_builder(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
task_executive_builder::tasking_map task_executive_builder::depth2_tasks_create(
    const config::depth2::controller_repository& config_repo,
    cta::ds::bi_tdgraph* const graph,
    rmath::rng* rng) {
  auto* task_config = config_repo.config_get<cta::config::task_alloc_config>();
  auto* exp_config = config_repo.config_get<config::exploration_config>();
  auto cache_color = carepr::light_type_index()[carepr::light_type_index::kCache];

  fsm::expstrat::block_factory block_factory;
  fsm::expstrat::cache_factory cache_factory;
  fsm::expstrat::foraging_expstrat::params expstrat_cachep(saa(),
                                                           nullptr,
                                                           cache_sel_matrix(),
                                                           perception()->dpo_store(),
                                                           cache_color);
  fsm::expstrat::foraging_expstrat::params expstrat_blockp(saa(),
                                                           nullptr,
                                                           cache_sel_matrix(),
                                                           perception()->dpo_store(),
                                                           rutils::color());

  fsm::fsm_ro_params params = {.bsel_matrix = block_sel_matrix(),
                               .csel_matrix = cache_sel_matrix(),
                               .store = perception()->dpo_store(),
                               .exp_config = *exp_config};
  auto cache_starter_fsm =
      std::make_unique<fsm::depth2::block_to_cache_site_fsm>(
          &params,
          saa(),
          block_factory.create(exp_config->block_strategy, &expstrat_blockp, rng),
          rng);

  auto cache_finisher_fsm =
      std::make_unique<fsm::depth2::block_to_new_cache_fsm>(
          &params,
          saa(),
          block_factory.create(exp_config->block_strategy, &expstrat_blockp, rng),
          rng);

  auto cache_transferer_fsm =
      std::make_unique<fsm::depth2::cache_transferer_fsm>(
          &params,
          saa(),
          cache_factory.create(exp_config->cache_strategy, &expstrat_cachep, rng),
          rng);

  auto cache_collector_fsm =
      std::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          &params,
          saa(),
          cache_factory.create(exp_config->cache_strategy, &expstrat_cachep, rng),
          rng);

  auto cache_starter = std::make_unique<tasks::depth2::cache_starter>(
      task_config, std::move(cache_starter_fsm));
  auto cache_finisher = std::make_unique<tasks::depth2::cache_finisher>(
      task_config, std::move(cache_finisher_fsm));
  auto cache_transferer = std::make_unique<tasks::depth2::cache_transferer>(
      task_config, std::move(cache_transferer_fsm));
  auto cache_collector = std::make_unique<tasks::depth2::cache_collector>(
      task_config, std::move(cache_collector_fsm));

  auto collector =
      graph->find_vertex(tasks::depth1::foraging_task::kCollectorName);
  auto harvester =
      graph->find_vertex(tasks::depth1::foraging_task::kHarvesterName);

  collector->set_partitionable(true);
  collector->set_atomic(false);
  harvester->set_partitionable(true);
  harvester->set_atomic(false);
  cta::ds::bi_tdgraph::vertex_vector children1;
  children1.push_back(std::move(cache_starter));
  children1.push_back(std::move(cache_finisher));
  cta::ds::bi_tdgraph::vertex_vector children2;
  children2.push_back(std::move(cache_transferer));
  children2.push_back(std::move(cache_collector));

  graph->install_tab(tasks::depth1::foraging_task::kHarvesterName,
                     std::move(children1),
                     rng);
  graph->install_tab(tasks::depth1::foraging_task::kCollectorName,
                     std::move(children2),
                     rng);
  return tasking_map{
      {"cache_starter",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheStarterName)},
      {"cache_finisher",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheFinisherName)},
      {"cache_transferer",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheTransfererName)},
      {"cache_collector",
       graph->find_vertex(tasks::depth2::foraging_task::kCacheCollectorName)}};
} /* depth2_tasks_create() */

void task_executive_builder::depth2_exec_est_init(
    const config::depth2::controller_repository& config_repo,
    const tasking_map& map,
    cta::ds::bi_tdgraph* graph,
    rmath::rng* rng) {
  auto* task_config = config_repo.config_get<cta::config::task_alloc_config>();

  auto cache_starter = map.find("cache_starter")->second;
  auto cache_finisher = map.find("cache_finisher")->second;
  auto cache_transferer = map.find("cache_transferer")->second;
  auto cache_collector = map.find("cache_collector")->second;
  if (!task_config->exec_est.seed_enabled) {
    return;
  }
  /*
   * As part of seeding exec estimates, we set the last executed subtask for a
   * TAB. Collector, harvester not partitionable in depth 1 initialization, so
   * they have only been initialized as atomic tasks.
   */
  if (0 == rng->uniform(0, 1)) {
    graph->tab_child(graph->root_tab(), graph->root_tab()->child1())
        ->last_subtask(cache_starter);
    graph->tab_child(graph->root_tab(), graph->root_tab()->child2())
        ->last_subtask(cache_transferer);
  } else {
    graph->tab_child(graph->root_tab(), graph->root_tab()->child1())
        ->last_subtask(cache_finisher);
    graph->tab_child(graph->root_tab(), graph->root_tab()->child2())
        ->last_subtask(cache_collector);
  }
  rmath::rangeu cs_bounds =
      task_config->exec_est.ranges.find("cache_starter")->second;
  rmath::rangeu cf_bounds =
      task_config->exec_est.ranges.find("cache_finisher")->second;
  rmath::rangeu ct_bounds =
      task_config->exec_est.ranges.find("cache_transferer")->second;
  rmath::rangeu cc_bounds =
      task_config->exec_est.ranges.find("cache_collector")->second;

  ER_INFO("Seeding exec estimate for tasks: '%s'=%s, '%s'=%s",
          cache_starter->name().c_str(),
          cs_bounds.to_str().c_str(),
          cache_finisher->name().c_str(),
          cf_bounds.to_str().c_str());
  cache_starter->exec_estimate_init(cs_bounds, rng);
  cache_finisher->exec_estimate_init(cf_bounds, rng);

  ER_INFO("Seeding exec estimate for tasks: '%s'=%s, '%s'=%s",
          cache_transferer->name().c_str(),
          ct_bounds.to_str().c_str(),
          cache_collector->name().c_str(),
          cc_bounds.to_str().c_str());
  cache_transferer->exec_estimate_init(ct_bounds, rng);
  cache_collector->exec_estimate_init(cc_bounds, rng);
} /* depth2_exec_est_init() */

void task_executive_builder::depth2_subtasks_init(
    const tasking_map& map,
    cta::ds::bi_tdgraph* graph,
    rmath::rng* rng) {

  auto cache_starter = map.find("cache_starter")->second;
  auto cache_finisher = map.find("cache_finisher")->second;
  auto cache_transferer = map.find("cache_transferer")->second;
  auto cache_collector = map.find("cache_collector")->second;

  /*
   * As part of seeding exec estimates, we set the last executed subtask for a
   * TAB. Collector, harvester not partitionable in depth 1 initialization, so
   * they have only been initialized as atomic tasks.
   */
  if (0 == rng->uniform(0, 1)) {
    graph->tab_child(graph->root_tab(), graph->root_tab()->child1())
        ->last_subtask(cache_starter);
    graph->tab_child(graph->root_tab(), graph->root_tab()->child2())
        ->last_subtask(cache_transferer);
  } else {
    graph->tab_child(graph->root_tab(), graph->root_tab()->child1())
        ->last_subtask(cache_finisher);
    graph->tab_child(graph->root_tab(), graph->root_tab()->child2())
        ->last_subtask(cache_collector);
  }
} /* depth2_subtasks_init() */

std::unique_ptr<cta::bi_tdgraph_executive> task_executive_builder::operator()(
    const config::depth2::controller_repository& config_repo,
    rmath::rng* rng) {
  auto* task_config = config_repo.config_get<cta::config::task_alloc_config>();
  auto variant =
      std::make_unique<cta::ds::ds_variant>(cta::ds::bi_tdgraph(task_config));
  auto graph = boost::get<cta::ds::bi_tdgraph>(variant.get());
  const auto* execp =
      config_repo.config_get<cta::config::task_executive_config>();
  const auto* allocp = config_repo.config_get<cta::config::task_alloc_config>();

  /* can be omitted if the user wants the default values */
  if (nullptr == execp) {
    execp = std::make_unique<cta::config::task_executive_config>().get();
  }

  auto map1 = depth1_tasks_create(config_repo, graph, rng);
  depth1_exec_est_init(config_repo, map1, graph, rng);
  depth1_subtasks_init(map1, graph, rng);

  auto map2 = depth2_tasks_create(config_repo, graph, rng);
  depth2_exec_est_init(config_repo, map2, graph, rng);
  depth2_subtasks_init(map2, graph, rng);

  /*
   * Only necessary if we are using the stochastic neighborhood policy; causes
   * segfaults due to asserts otherwise.
   */
  if (cta::bi_tdgraph_allocator::kPolicyStochNBHD1 == allocp->policy) {
    graph->active_tab_init(allocp->stoch_nbhd1.tab_init_policy, rng);
  }
  return std::make_unique<cta::bi_tdgraph_executive>(
      execp, allocp, std::move(variant), rng);
} /* initialize() */

NS_END(depth2, controller, fordyca);
