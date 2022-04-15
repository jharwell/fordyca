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
#include "fordyca/controller/cognitive/d2/task_executive_builder.hpp"

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/strategy/nest_acq/factory.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/ta/bi_tdgraph_allocator.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/config/task_alloc_config.hpp"
#include "cosm/ta/config/task_executive_config.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"

#include "fordyca/controller/config/d2/controller_repository.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/d2/block_to_cache_site_fsm.hpp"
#include "fordyca/fsm/d2/block_to_new_cache_fsm.hpp"
#include "fordyca/fsm/d2/cache_transferer_fsm.hpp"
#include "fordyca/strategy/config/strategy_config.hpp"
#include "fordyca/strategy/explore/block_factory.hpp"
#include "fordyca/strategy/explore/cache_factory.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/tasks/d1/collector.hpp"
#include "fordyca/tasks/d2/cache_collector.hpp"
#include "fordyca/tasks/d2/cache_finisher.hpp"
#include "fordyca/tasks/d2/cache_starter.hpp"
#include "fordyca/tasks/d2/cache_transferer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
task_executive_builder::task_executive_builder(
    const controller::cognitive::block_sel_matrix* bsel_matrix,
    const controller::cognitive::cache_sel_matrix* csel_matrix,
    cspatial::interference_tracker* inta,
    cspatial::nest_zone_tracker* nz,
    csubsystem::saa_subsystemQ3D* const saa,
    fsperception::foraging_perception_subsystem* const perception)
    : d1::task_executive_builder(bsel_matrix,
                                 csel_matrix,
                                 inta,
                                 nz,
                                 saa,
                                 perception),
      ER_CLIENT_INIT("fordyca.controller.d2.task_executive_builder") {}

task_executive_builder::~task_executive_builder(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
task_executive_builder::tasking_map task_executive_builder::d2_tasks_create(
    const config::d2::controller_repository& config_repo,
    cta::ds::bi_tdgraph* const graph,
    rmath::rng* rng) {
  const auto* task_config =
      config_repo.config_get<cta::config::task_alloc_config>();
  const auto* strat_config = config_repo.config_get<fsconfig::strategy_config>();
  auto cache_color = carepr::light_type_index()[carepr::light_type_index::kCache];

  fsexplore::block_factory block_factory;
  fsexplore::cache_factory cache_factory;
  csstrategy::nest_acq::factory nest_acq_factory;
  csfsm::fsm_params fsm_params{
    saa(),
    inta_tracker(),
    nz_tracker(),
  };
  auto strategy_cachep = fstrategy::strategy_params{
    .fsm =&fsm_params,
    .explore = &strat_config->caches.explore,
    .bsel_matrix = nullptr,
    .csel_matrix = cache_sel_matrix(),
    .accessor = perception()->known_objects(),
    .ledtaxis_target = cache_color
  };
  auto strategy_blockp = fstrategy::strategy_params{
    .fsm = &fsm_params,
    .explore = &strat_config->blocks.explore,
    .bsel_matrix = nullptr,
    .csel_matrix = cache_sel_matrix(),
    .accessor = perception()->known_objects(),
    .ledtaxis_target = rutils::color()
  };
  fsm::fsm_ro_params params = { .bsel_matrix = block_sel_matrix(),
                                .csel_matrix = cache_sel_matrix(),
                                .store = perception()->model<fspds::dpo_store>(),
                                .accessor = perception()->known_objects(),
                                .strategy = *strat_config };

  auto cache_starter_fsm = std::make_unique<fsm::d2::block_to_cache_site_fsm>(
      &params,
      &fsm_params,
      block_factory.create(
          strat_config->blocks.explore.strategy, &strategy_blockp, rng),
      rng);

  auto cache_finisher_fsm = std::make_unique<fsm::d2::block_to_new_cache_fsm>(
      &params,
      &fsm_params,
      block_factory.create(
          strat_config->blocks.explore.strategy, &strategy_blockp, rng),
      rng);

  auto cache_transferer_fsm = std::make_unique<fsm::d2::cache_transferer_fsm>(
      &params,
      &fsm_params,
      cache_factory.create(
          strat_config->caches.explore.strategy, &strategy_cachep, rng),
      rng);

  auto cache_collector_fsm = std::make_unique<fsm::d1::cached_block_to_nest_fsm>(
      &params,
      &fsm_params,
      cache_factory.create(
          strat_config->caches.explore.strategy, &strategy_cachep, rng),
      nest_acq_factory.create(strat_config->nest_acq.strategy,
                              &strat_config->nest_acq,
                              &fsm_params,
                              rng),
      rng);

  auto cache_starter = std::make_unique<tasks::d2::cache_starter>(
      task_config, std::move(cache_starter_fsm));
  auto cache_finisher = std::make_unique<tasks::d2::cache_finisher>(
      task_config, std::move(cache_finisher_fsm));
  auto cache_transferer = std::make_unique<tasks::d2::cache_transferer>(
      task_config, std::move(cache_transferer_fsm));
  auto cache_collector = std::make_unique<tasks::d2::cache_collector>(
      task_config, std::move(cache_collector_fsm));

  auto* collector = graph->find_vertex(tasks::d1::foraging_task::kCollectorName);
  auto* harvester = graph->find_vertex(tasks::d1::foraging_task::kHarvesterName);

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

  graph->install_tab(
      tasks::d1::foraging_task::kHarvesterName, std::move(children1), rng);
  graph->install_tab(
      tasks::d1::foraging_task::kCollectorName, std::move(children2), rng);
  return tasking_map{
    { ftd2::foraging_task::kCacheStarterName,
      graph->find_vertex(tasks::d2::foraging_task::kCacheStarterName) },
    { ftd2::foraging_task::kCacheFinisherName,
      graph->find_vertex(tasks::d2::foraging_task::kCacheFinisherName) },
    { ftd2::foraging_task::kCacheTransfererName,
      graph->find_vertex(tasks::d2::foraging_task::kCacheTransfererName) },
    { ftd2::foraging_task::kCacheCollectorName,
      graph->find_vertex(tasks::d2::foraging_task::kCacheCollectorName) }
  };
} /* d2_tasks_create() */

void task_executive_builder::d2_exec_est_init(
    const config::d2::controller_repository& config_repo,
    const tasking_map& map,
    cta::ds::bi_tdgraph* graph,
    rmath::rng* rng) {
  const auto* task_config =
      config_repo.config_get<cta::config::task_alloc_config>();

  auto* cache_starter = map.find(ftd2::foraging_task::kCacheStarterName)->second;
  auto* cache_finisher =
      map.find(ftd2::foraging_task::kCacheFinisherName)->second;
  auto* cache_transferer =
      map.find(ftd2::foraging_task::kCacheTransfererName)->second;
  auto* cache_collector =
      map.find(ftd2::foraging_task::kCacheCollectorName)->second;
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
  rmath::rangez cs_bounds =
      task_config->exec_est.ranges.find(ftd2::foraging_task::kCacheStarterName)
          ->second;
  rmath::rangez cf_bounds =
      task_config->exec_est.ranges.find(ftd2::foraging_task::kCacheFinisherName)
          ->second;
  rmath::rangez ct_bounds = task_config->exec_est.ranges
                                .find(ftd2::foraging_task::kCacheTransfererName)
                                ->second;
  rmath::rangez cc_bounds =
      task_config->exec_est.ranges.find(ftd2::foraging_task::kCacheCollectorName)
          ->second;

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
} /* d2_exec_est_init() */

void task_executive_builder::d2_subtasks_init(const tasking_map& map,
                                              cta::ds::bi_tdgraph* graph,
                                              rmath::rng* rng) {
  auto* cache_starter = map.find(ftd2::foraging_task::kCacheStarterName)->second;
  auto* cache_finisher =
      map.find(ftd2::foraging_task::kCacheFinisherName)->second;
  auto* cache_transferer =
      map.find(ftd2::foraging_task::kCacheTransfererName)->second;
  auto* cache_collector =
      map.find(ftd2::foraging_task::kCacheCollectorName)->second;

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
} /* d2_subtasks_init() */

std::unique_ptr<cta::bi_tdgraph_executive> task_executive_builder::operator()(
    const config::d2::controller_repository& config_repo,
    rmath::rng* rng) {
  const auto* task_config =
      config_repo.config_get<cta::config::task_alloc_config>();
  cta::ds::ds_variant variant =
      std::make_unique<cta::ds::bi_tdgraph>(task_config);
  auto* graph = std::get<std::unique_ptr<cta::ds::bi_tdgraph>>(variant).get();

  const auto* execp =
      config_repo.config_get<cta::config::task_executive_config>();
  const auto* allocp = config_repo.config_get<cta::config::task_alloc_config>();

  /* can be omitted if the user wants the default values */
  if (nullptr == execp) {
    execp = std::make_unique<cta::config::task_executive_config>().get();
  }

  auto map1 = d1_tasks_create(config_repo, graph, rng);
  d1_exec_est_init(config_repo, map1, graph, rng);
  d1_subtasks_init(map1, graph, rng);

  auto map2 = d2_tasks_create(config_repo, graph, rng);
  d2_exec_est_init(config_repo, map2, graph, rng);
  d2_subtasks_init(map2, graph, rng);

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

NS_END(cognitive, d2, controller, fordyca);
