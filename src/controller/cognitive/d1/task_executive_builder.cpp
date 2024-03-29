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
#include "fordyca/controller/cognitive/d1/task_executive_builder.hpp"

#include <vector>

#include "cosm/arena/repr/base_cache.hpp"
#include "cosm/arena/repr/light_type_index.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/spatial/strategy/nest/acq/factory.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/ta/bi_tdgraph_allocator.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/config/task_alloc_config.hpp"
#include "cosm/ta/config/task_executive_config.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"
#include "cosm/spatial/strategy/nest/exit/factory.hpp"
#include "cosm/spatial/strategy/blocks/drop/factory.hpp"

#include "fordyca/controller/config/d1/controller_repository.hpp"
#include "fordyca/fsm/d0/dpo_fsm.hpp"
#include "fordyca/fsm/d1/block_to_existing_cache_fsm.hpp"
#include "fordyca/fsm/d1/cached_block_to_nest_fsm.hpp"
#include "fordyca/strategy/config/strategy_config.hpp"
#include "fordyca/strategy/explore/block_factory.hpp"
#include "fordyca/strategy/explore/cache_factory.hpp"
#include "fordyca/subsystem/perception/dpo_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/ds/dpo_store.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"
#include "fordyca/subsystem/perception/mdpo_perception_subsystem.hpp"
#include "fordyca/tasks/d0/generalist.hpp"
#include "fordyca/tasks/d1/collector.hpp"
#include "fordyca/tasks/d1/harvester.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1);

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
    : ER_CLIENT_INIT("fordyca.controller.d1.task_executive_builder"),
      mc_csel_matrix(csel_matrix),
      mc_bsel_matrix(bsel_matrix),
      m_inta_tracker(inta),
      m_nz_tracker(nz),
      m_saa(saa),
      m_perception(perception) {}

task_executive_builder::~task_executive_builder(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
task_executive_builder::tasking_map task_executive_builder::d1_tasks_create(
    const config::d1::controller_repository& config_repo,
    cta::ds::bi_tdgraph* const graph,
    rmath::rng* rng) {
  const auto* task_config =
      config_repo.config_get<cta::config::task_alloc_config>();
  const auto* strat_config = config_repo.config_get<fsconfig::strategy_config>();
  auto cache_color = carepr::light_type_index()[carepr::light_type_index::kCache];
  csfsm::fsm_params fsm_params{
    saa(),
    inta_tracker(),
    nz_tracker(),
  };
  auto strategy_cachep = fstrategy::strategy_params {
    .fsm =&fsm_params,
    .explore = &strat_config->caches.explore,
    .bsel_matrix = nullptr,
    .csel_matrix = mc_csel_matrix,
    .accessor = m_perception->known_objects(),
    .ledtaxis_target = cache_color
  };
  auto strategy_blockp = fstrategy::strategy_params {
    .fsm =&fsm_params,
    .explore = &strat_config->blocks.explore,
    .bsel_matrix = nullptr,
    .csel_matrix = mc_csel_matrix,
    .accessor = m_perception->known_objects(),
    .ledtaxis_target = rutils::color()
  };

  ER_ASSERT(nullptr != mc_bsel_matrix, "NULL block selection matrix");
  ER_ASSERT(nullptr != mc_csel_matrix, "NULL cache selection matrix");

  fsexplore::block_factory block_search_factory;
  fsexplore::cache_factory cache_search_factory;
  cssnest::acq::factory nest_acq_factory;
  cssnest::exit::factory nest_exit_factory;
  cssblocks::drop::factory block_drop_factory;

  fsm::fsm_ro_params ro_params = { .bsel_matrix = block_sel_matrix(),
                                   .csel_matrix = mc_csel_matrix,
                                   .store =
                                   perception()->model<fspds::dpo_store>(),
                                   .accessor = m_perception->known_objects(),
                                   .strategy = *strat_config };

  cffsm::strategy_set generalist_strategies = {
    .explore = block_search_factory.create(strat_config->blocks.explore.strategy,
                                    &strategy_blockp,
                                    rng),
    .nest_acq = nest_acq_factory.create(strat_config->nest.acq.strategy,
                                        &strat_config->nest.acq,
                                        &fsm_params,
                                        rng),
    .nest_exit = nest_exit_factory.create(strat_config->nest.exit.strategy,
                                         &strat_config->nest.exit,
                                         &fsm_params,
                                         rng),
    .block_drop = block_drop_factory.create(strat_config->blocks.drop.strategy,
                                            &fsm_params,
                                            &strat_config->blocks.drop,
                                            rng),
  };
  auto generalist_fsm = std::make_unique<fsm::d0::free_block_to_nest_fsm>(
      &ro_params,
      &fsm_params,
      std::move(generalist_strategies),
      rng);

  cffsm::strategy_set collector_strategies = {
    .explore = cache_search_factory.create(strat_config->caches.explore.strategy,
                                           &strategy_cachep,
                                           rng),
    .nest_acq = nest_acq_factory.create(strat_config->nest.acq.strategy,
                                        &strat_config->nest.acq,
                                        &fsm_params,
                                        rng),
    .nest_exit = nest_exit_factory.create(strat_config->nest.exit.strategy,
                                          &strat_config->nest.exit,
                                          &fsm_params,
                                          rng),
    .block_drop = block_drop_factory.create(strat_config->blocks.drop.strategy,
                                            &fsm_params,
                                            &strat_config->blocks.drop,
                                            rng),
  };
  auto collector_fsm = std::make_unique<fsm::d1::cached_block_to_nest_fsm>(
      &ro_params,
      &fsm_params,
      std::move(collector_strategies),
      rng);

  auto harvester_fsm = std::make_unique<fsm::d1::block_to_existing_cache_fsm>(
      &ro_params, &fsm_params, rng);

  auto collector = std::make_unique<tasks::d1::collector>(
      task_config, std::move(collector_fsm));

  auto harvester = std::make_unique<tasks::d1::harvester>(
      task_config, std::move(harvester_fsm));
  auto generalist = std::make_unique<tasks::d0::generalist>(
      task_config, std::move(generalist_fsm));
  generalist->set_partitionable(true);
  generalist->set_atomic(false);

  graph->set_root(std::move(generalist));
  cta::ds::bi_tdgraph::vertex_vector children;
  children.push_back(std::move(harvester));
  children.push_back(std::move(collector));
  graph->install_tab(
      tasks::d0::foraging_task::kGeneralistName, std::move(children), rng);
  return tasking_map{
    { tasks::d0::foraging_task::kGeneralistName,
      graph->find_vertex(tasks::d0::foraging_task::kGeneralistName) },
    { tasks::d1::foraging_task::kCollectorName,
      graph->find_vertex(tasks::d1::foraging_task::kCollectorName) },
    { tasks::d1::foraging_task::kHarvesterName,
      graph->find_vertex(tasks::d1::foraging_task::kHarvesterName) }
  };
} /* d1_tasks_create() */

void task_executive_builder::d1_exec_est_init(
    const config::d1::controller_repository& config_repo,
    const tasking_map& map,
    cta::ds::bi_tdgraph* const graph,
    rmath::rng* rng) {
  const auto* task_config =
      config_repo.config_get<cta::config::task_alloc_config>();
  if (!task_config->exec_est.seed_enabled) {
    return;
  }
  /*
   * Generalist is not partitionable in depth 0 initialization, so this has
   * not been done.
   */
  auto* harvester = map.find(ftd1::foraging_task::kHarvesterName)->second;
  auto* collector = map.find(ftd1::foraging_task::kCollectorName)->second;
  auto* generalist = map.find(ftd0::foraging_task::kGeneralistName)->second;

  if (0 == rng->uniform(0, 1)) {
    graph->root_tab()->last_subtask(harvester);
  } else {
    graph->root_tab()->last_subtask(collector);
  }

  rmath::rangez g_bounds =
      task_config->exec_est.ranges.find(ftd0::foraging_task::kGeneralistName)
          ->second;

  auto h_bounds = task_config->exec_est.ranges.find("harvester")->second;
  auto c_bounds = task_config->exec_est.ranges.find("collector")->second;
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
} /* d1_exec_est_init() */

void task_executive_builder::d1_subtasks_init(const tasking_map& map,
                                              cta::ds::bi_tdgraph* const graph,
                                              rmath::rng* rng) {
  /*
   * Generalist is not partitionable in depth 0 initialization, so this has
   * not been done.
   */
  auto* harvester = map.find(ftd1::foraging_task::kHarvesterName)->second;
  auto* collector = map.find(ftd1::foraging_task::kCollectorName)->second;

  if (0 == rng->uniform(0, 1)) {
    graph->root_tab()->last_subtask(harvester);
  } else {
    graph->root_tab()->last_subtask(collector);
  }
} /* d1_subtasks_init() */

std::unique_ptr<cta::bi_tdgraph_executive> task_executive_builder::operator()(
    const config::d1::controller_repository& config_repo,
    rmath::rng* rng) {
  const auto* task_config =
      config_repo.config_get<cta::config::task_alloc_config>();
  cta::ds::ds_variant variant =
      std::make_unique<cta::ds::bi_tdgraph>(task_config);

  auto* graph = std::get<std::unique_ptr<cta::ds::bi_tdgraph>>(variant).get();
  auto map = d1_tasks_create(config_repo, graph, rng);
  const auto* execp =
      config_repo.config_get<cta::config::task_executive_config>();
  const auto* allocp = config_repo.config_get<cta::config::task_alloc_config>();

  /* can be omitted if the user wants the default values */
  if (nullptr == execp) {
    execp = std::make_unique<cta::config::task_executive_config>().get();
  }

  /*
   * Only necessary if we are using the stochastic neighborhood policy; causes
   * segfaults due to asserts otherwise.
   */
  if (cta::bi_tdgraph_allocator::kPolicyStochNBHD1 == allocp->policy) {
    graph->active_tab_init(allocp->stoch_nbhd1.tab_init_policy, rng);
  }
  d1_exec_est_init(config_repo, map, graph, rng);
  d1_subtasks_init(map, graph, rng);

  return std::make_unique<cta::bi_tdgraph_executive>(
      execp, allocp, std::move(variant), rng);
} /* initialize() */

NS_END(cognitive, d1, controller, fordyca);
