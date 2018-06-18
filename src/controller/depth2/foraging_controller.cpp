/**
 * @file foraging_controller.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "fordyca/controller/depth2/foraging_controller.hpp"
#include <fstream>

#include "fordyca/controller/actuation_subsystem.hpp"
#include "fordyca/controller/depth1/perception_subsystem.hpp"
#include "fordyca/controller/depth1/sensing_subsystem.hpp"
#include "fordyca/controller/saa_subsystem.hpp"
#include "fordyca/params/depth2/exec_estimates_params.hpp"
#include "fordyca/params/depth1/task_repository.hpp"
#include "fordyca/params/depth2/task_repository.hpp"
#include "fordyca/params/depth0/stateful_foraging_repository.hpp"
#include "fordyca/params/fsm_params.hpp"
#include "fordyca/params/sensing_params.hpp"
#include "fordyca/representation/base_cache.hpp"
#include "fordyca/representation/perceived_arena_map.hpp"
#include "fordyca/tasks/depth0/generalist.hpp"
#include "fordyca/tasks/depth1/collector.hpp"
#include "fordyca/tasks/depth1/harvester.hpp"
#include "fordyca/tasks/depth2/cache_starter.hpp"
#include "fordyca/tasks/depth2/cache_finisher.hpp"
#include "fordyca/tasks/depth2/cache_transferer.hpp"
#include "fordyca/fsm/depth0/stateful_foraging_fsm.hpp"
#include "fordyca/fsm/depth1/block_to_existing_cache_fsm.hpp"
#include "fordyca/fsm/depth1/cached_block_to_nest_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_new_cache_fsm.hpp"
#include "fordyca/fsm/depth2/block_to_cache_site_fsm.hpp"
#include "fordyca/fsm/depth2/cache_transferer_fsm.hpp"

#include "rcppsw/er/server.hpp"
#include "rcppsw/task_allocation/polled_executive.hpp"
#include "rcppsw/task_allocation/task_decomposition_graph.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, depth2);
using representation::occupancy_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_controller::foraging_controller(void)
    : depth1::foraging_controller(), m_executive() {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_controller::ControlStep(void) {
  perception()->update(depth0::stateful_foraging_controller::los());

  saa_subsystem()->actuation()->block_throttle_toggle(is_carrying_block());
  saa_subsystem()->actuation()->block_throttle_update();

  m_executive->run();
} /* ControlStep() */

void foraging_controller::Init(ticpp::Element& node) {
  /*
   * Note that we do not call \ref depth1::foraging_controller::Init()--there
   * is nothing in there that we need.
   */
  base_foraging_controller::Init(node);

  ER_NOM("Initializing depth2 foraging controller");
  params::depth1::task_repository depth1_task_repo(client::server_ref());
  params::depth2::task_repository depth2_task_repo(client::server_ref());
  params::depth0::stateful_foraging_repository stateful_repo(client::server_ref());

  depth1_task_repo.parse_all(node);
  server_handle()->log_stream() << depth1_task_repo;
  depth2_task_repo.parse_all(node);
  server_handle()->log_stream() << depth2_task_repo;
  stateful_repo.parse_all(node);
  server_handle()->log_stream() << stateful_repo;

  ER_ASSERT(depth1_task_repo.validate_all(),
            "FATAL: Not all depth1 task parameters were validated");
  ER_ASSERT(depth2_task_repo.validate_all(),
            "FATAL: Not all depth2 task parameters were validated");
  ER_ASSERT(stateful_repo.validate_all(),
            "FATAL: Not all stateful foraging parameters were validated");

  /* Put in new depth1 sensors and perception, ala strategy pattern */
  saa_subsystem()->sensing(std::make_shared<depth1::sensing_subsystem>(
      stateful_repo.parse_results<struct params::sensing_params>(),
      &saa_subsystem()->sensing()->sensor_list()));

  perception(rcppsw::make_unique<depth1::perception_subsystem>(
      client::server_ref(),
      stateful_repo.parse_results<params::perception_params>(),
      GetId()));

  /* initialize tasking */
  tasking_init(&stateful_repo, &depth1_task_repo, &depth2_task_repo);

  ER_NOM("depth2 foraging controller initialization finished");
} /* Init() */

void foraging_controller::depth0_tasking_init(
    params::depth0::stateful_foraging_repository* const stateful_repo,
    params::depth1::task_repository* const task_repo) {
  auto* exec_params = task_repo->parse_results<ta::executive_params>();
  auto* est_params = task_repo->parse_results<params::depth2::exec_estimates_params>();

  std::unique_ptr<ta::taskable> generalist_fsm =
      rcppsw::make_unique<fsm::depth0::stateful_foraging_fsm>(
          stateful_repo->parse_results<params::fsm_params>(),
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());

  auto generalist =
      ta::make_task_graph_vertex<tasks::depth0::generalist>(exec_params,
                                                            generalist_fsm);

  if (est_params->enabled) {
    std::static_pointer_cast<ta::polled_task>(generalist)
        ->init_random(est_params->generalist_range.GetMin(),
                      est_params->generalist_range.GetMax());
  }

  m_graph =
      std::make_shared<ta::task_decomposition_graph>(client::server_ref());

  m_graph->set_root(generalist);
  generalist->set_partitionable();
} /* tasking_init() */

void foraging_controller::depth1_tasking_init(
    params::depth0::stateful_foraging_repository* const stateful_repo,
    params::depth1::task_repository* const task_repo) {
  auto* exec_params = task_repo->parse_results<ta::executive_params>();
  auto* est_params = task_repo->parse_results<params::depth2::exec_estimates_params>();

  std::unique_ptr<ta::taskable> collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          stateful_repo->parse_results<params::fsm_params>(),
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());

  std::unique_ptr<ta::taskable> harvester_fsm =
      rcppsw::make_unique<fsm::depth1::block_to_existing_cache_fsm>(
          stateful_repo->parse_results<params::fsm_params>(),
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
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

  harvester->set_partitionable();
  collector->set_partitionable();

  m_graph->set_children(tasks::depth0::foraging_task::kGeneralistName,
                        std::list<ta::task_graph_vertex>({collector,
                                harvester}));
} /* depth1_tasking_init() */

void foraging_controller::depth2_tasking_init(
    params::depth0::stateful_foraging_repository* const stateful_repo,
    params::depth2::task_repository* const task_repo) {

  auto* est_params = task_repo->parse_results<params::depth2::exec_estimates_params>();
  auto* exec_params = task_repo->parse_results<ta::executive_params>();

  std::unique_ptr<ta::taskable> cache_starter_fsm =
      rcppsw::make_unique<fsm::depth2::block_to_cache_site_fsm>(
          stateful_repo->parse_results<params::fsm_params>(),
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());

  std::unique_ptr<ta::taskable> cache_finisher_fsm =
      rcppsw::make_unique<fsm::depth2::block_to_new_cache_fsm>(
          stateful_repo->parse_results<params::fsm_params>(),
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());

  std::unique_ptr<ta::taskable> cache_transferer_fsm =
      rcppsw::make_unique<fsm::depth2::cache_transferer_fsm>(
          stateful_repo->parse_results<params::fsm_params>(),
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());
  std::unique_ptr<ta::taskable> cache_collector_fsm =
      rcppsw::make_unique<fsm::depth1::cached_block_to_nest_fsm>(
          stateful_repo->parse_results<params::fsm_params>(),
          client::server_ref(),
          base_foraging_controller::saa_subsystem(),
          perception()->map());

  auto cache_starter = ta::make_task_graph_vertex<tasks::depth2::cache_starter>(
      exec_params,
      cache_starter_fsm);
  auto cache_finisher = ta::make_task_graph_vertex<tasks::depth2::cache_finisher>(
      exec_params,
      cache_finisher_fsm);
  auto cache_transferer = ta::make_task_graph_vertex<tasks::depth2::cache_transferer>(
      exec_params,
      cache_transferer_fsm);
  auto cache_collector = ta::make_task_graph_vertex<tasks::depth1::collector>(
      exec_params,
      cache_collector_fsm);

  if (est_params->enabled) {
    std::static_pointer_cast<ta::polled_task>(cache_starter)->init_random(
        est_params->cache_starter_range.GetMin(),
        est_params->cache_starter_range.GetMax());
    std::static_pointer_cast<ta::polled_task>(cache_finisher)->init_random(
        est_params->cache_finisher_range.GetMin(),
        est_params->cache_starter_range.GetMax());
    std::static_pointer_cast<ta::polled_task>(cache_transferer)->init_random(
        est_params->cache_transferer_range.GetMin(),
        est_params->cache_transferer_range.GetMax());
    std::static_pointer_cast<ta::polled_task>(cache_collector)->init_random(
        est_params->cache_collector_range.GetMin(),
        est_params->cache_collector_range.GetMax());
  }

  m_graph->set_children(tasks::depth1::foraging_task::kHarvesterName,
                      std::list<ta::task_graph_vertex>({cache_starter,
                              cache_finisher}));
  m_graph->set_children(tasks::depth1::foraging_task::kCollectorName,
                      std::list<ta::task_graph_vertex>({cache_transferer,
                              cache_collector}));
} /* depth2_tasking_init() */

void foraging_controller::tasking_init(
    params::depth0::stateful_foraging_repository *const stateful_repo,
    params::depth1::task_repository *const depth1_task_repo,
    params::depth2::task_repository *const depth2_task_repo) {

  depth0_tasking_init(stateful_repo, depth1_task_repo);
  depth1_tasking_init(stateful_repo, depth1_task_repo);
  depth2_tasking_init(stateful_repo, depth2_task_repo);

  m_executive =
      rcppsw::make_unique<ta::polled_executive>(client::server_ref(), m_graph);
} /* tasking_init() */

__rcsw_pure std::shared_ptr<tasks::base_foraging_task> foraging_controller::current_task(
    void) const {
  return std::dynamic_pointer_cast<tasks::base_foraging_task>(
      m_executive->current_task());
} /* current_task() */

/*******************************************************************************
 * Task Metrics
 ******************************************************************************/
bool foraging_controller::employed_partitioning(void) const {
  ER_ASSERT(nullptr != current_task(),
            "FATAL: Have not yet employed partitioning?");

  auto task = std::dynamic_pointer_cast<ta::executable_task>(current_task());
  auto partitionable = std::dynamic_pointer_cast<ta::partitionable_task>(task);

  if (!task->is_partitionable()) {
    partitionable = std::dynamic_pointer_cast<ta::partitionable_task>(
        m_executive->parent_task(
            std::dynamic_pointer_cast<ta::executable_task>(current_task())));
  }
  ER_ASSERT(nullptr != partitionable, "FATAL: Not an executable task?");
  return partitionable->employed_partitioning();
} /* employed_partitioning() */

std::string foraging_controller::subtask_selection(void) const {
  ER_ASSERT(nullptr != current_task(),
            "FATAL: Have not yet selected a subtask?");
  /*
   * If we get into this function, then employed_partitioning() must have
   * returned TRUE, so we can just return the name of the current task, which
   * MUST be a subtask.
   */
  return current_task_name();
} /* subtask_selection() */

std::string foraging_controller::current_task_name(void) const {
  return std::dynamic_pointer_cast<ta::logical_task>(current_task())->name();
} /* current_task_name() */

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
using depth2_foraging_controller = foraging_controller;

REGISTER_CONTROLLER(depth2_foraging_controller,
                    "depth2_foraging_controller"); // NOLINT

NS_END(depth2, controller, fordyca);
