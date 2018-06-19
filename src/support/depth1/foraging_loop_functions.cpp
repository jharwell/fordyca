/**
 * @file foraging_loop_functions.cpp
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
#include "fordyca/support/depth1/foraging_loop_functions.hpp"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <random>

#include "fordyca/controller/depth1/foraging_controller.hpp"
#include "fordyca/math/cache_respawn_probability.hpp"
#include "fordyca/metrics/cache_metrics_collector.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics_collector.hpp"
#include "fordyca/metrics/tasks/execution_metrics_collector.hpp"
#include "fordyca/metrics/tasks/management_metrics_collector.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "fordyca/tasks/depth1/existing_cache_interactor.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"

#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(ticpp::Element& node) {
  depth0::stateful_foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth1 foraging loop functions");
  params::loop_function_repository repo(server_ref());

  repo.parse_all(node);
  rcppsw::er::g_server->log_stream() << repo;

  auto* arenap = repo.parse_results<params::arena_map_params>();
  /* initialize cache handling and create initial cache */
  cache_handling_init(arenap);

  /* initialize stat collecting */
  metric_collecting_init(repo.parse_results<struct params::output_params>());

  /* intitialize robot interactions with environment */
  m_interactor = rcppsw::make_unique<interactor>(rcppsw::er::g_server,
                                                 arena_map(),
                                                 floor(),
                                                 nest_xrange(),
                                                 nest_yrange(),
                                                 arenap->static_cache.usage_penalty);

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = dynamic_cast<controller::depth1::foraging_controller&>(
        robot.GetControllableEntity().GetController());
    controller.display_task(
        repo.parse_results<params::visualization_params>()->robot_task);
  } /* for(&entity..) */
  ER_NOM("depth1_foraging loop functions initialization finished");
}

void foraging_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
  auto& controller = dynamic_cast<controller::depth1::foraging_controller&>(
      robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  collector_group().collect(
      "fsm::distance", static_cast<metrics::fsm::distance_metrics&>(controller));
  collector_group().collect(
      "tasks::management",
      static_cast<rcppsw::metrics::tasks::management_metrics&>(controller));

  if (nullptr != controller.current_task()) {
    collector_group().collect_if(
        "fsm::block_acquisition",
        static_cast<metrics::fsm::goal_acquisition_metrics&>(
            *controller.current_task()),
        [&](const rcppsw::metrics::base_metrics& metrics) {
          return acquisition_goal_type::kBlock ==
                 static_cast<const metrics::fsm::goal_acquisition_metrics&>(
                     metrics)
                     .acquisition_goal();
        });
    collector_group().collect_if(
        "fsm::cache_acquisition",
        static_cast<metrics::fsm::goal_acquisition_metrics&>(
            *controller.current_task()),
        [&](const rcppsw::metrics::base_metrics& metrics) {
          return acquisition_goal_type::kExistingCache ==
                 static_cast<const metrics::fsm::goal_acquisition_metrics&>(
                     metrics)
                     .acquisition_goal();
        });

    collector_group().collect(
        "tasks::execution",
        static_cast<rcppsw::metrics::tasks::execution_metrics&>(
            *controller.current_task()));
  }

  /* send the robot its view of the world: what it sees and where it is */
  utils::set_robot_pos<decltype(controller)>(robot);
  set_robot_los<decltype(controller)>(robot, *arena_map());
  set_robot_tick<decltype(controller)>(robot);

  /* Now watch it react to the environment */
  (*m_interactor)(controller, GetSpace().GetSimulationClock());
} /* pre_step_iter() */

argos::CColor foraging_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  /* The nest is a light gray */
  if (nest_xrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      nest_yrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY70;
  }

  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks renderin inside of caches.
   */
  for (auto& cache : arena_map()->caches()) {
    if (cache->contains_point(plane_pos)) {
      return cache->color();
    }
  } /* for(&cache..) */

  for (auto& block : arena_map()->blocks()) {
    if (block->contains_point(plane_pos)) {
      return block->color();
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void foraging_loop_functions::PreStep() {
  /* Get metrics from caches */
  for (auto& c : arena_map()->caches()) {
    collector_group().collect("cache", static_cast<metrics::cache_metrics&>(*c));
    c->reset_metrics();
  } /* for(&c..) */

  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  pre_step_final();
} /* PreStep() */

void foraging_loop_functions::Reset() {
  collector_group().reset_all();
  arena_map()->static_cache_create();
}

void foraging_loop_functions::pre_step_final(void) {
  /*
   * The cache is recreated with a probability that depends on the relative
   * ratio between the # foragers and the # collectors. If there are more
   * foragers than collectors, then the cache will be recreated very quickly. If
   * there are more collectors than foragers, then it will probably not be
   * recreated immediately. And if there are no foragers, there is no chance
   * that the cache could be recreated (trying to emulate depth2 behavior here).
   */
  if (arena_map()->has_static_cache() && arena_map()->caches().empty()) {
    auto& collector = static_cast<metrics::tasks::execution_metrics_collector&>(
        *collector_group()["tasks::execution"]);
    int n_harvesters = collector.n_harvesters();
    int n_collectors = collector.n_collectors();
    math::cache_respawn_probability p(mc_cache_respawn_scale_factor);
    if (p.calc(n_harvesters, n_collectors) >=
        static_cast<double>(random()) / RAND_MAX) {
      arena_map()->static_cache_create();
      representation::cell2D& cell =
          arena_map()->access(arena_map()->caches()[0]->discrete_loc());
      ER_ASSERT(arena_map()->caches()[0]->n_blocks() == cell.block_count(),
                "FATAL: Cache/cell disagree on # of blocks: cache=%u/cell=%zu",
                arena_map()->caches()[0]->n_blocks(),
                cell.block_count());
      floor()->SetChanged();
    }
  }

  if (arena_map()->cache_removed()) {
    floor()->SetChanged();
    arena_map()->cache_removed(false);
  }

  collector_group().metrics_write_all(GetSpace().GetSimulationClock());
  collector_group().timestep_reset_all();
  collector_group().interval_reset_all();
  collector_group().timestep_inc_all();
} /* pre_step_final() */

void foraging_loop_functions::cache_handling_init(
    const struct params::arena_map_params* arenap) {
  /*
   * Regardless of how many foragers/etc there are, always create an
   * initial cache.
   */
  if (arenap->static_cache.enable) {
    arena_map()->static_cache_create();
  }

  mc_cache_respawn_scale_factor = arenap->static_cache.respawn_scale_factor;
} /* cache_handling_init() */

void foraging_loop_functions::metric_collecting_init(
    const struct params::output_params* output_p) {
  collector_group()
      .register_collector<metrics::fsm::goal_acquisition_metrics_collector>(
          "fsm::cache_acquisition",
          metrics_path() + "/" + output_p->metrics.cache_acquisition_fname,
          output_p->metrics.collect_interval);

  collector_group()
      .register_collector<metrics::tasks::execution_metrics_collector>(
          "tasks::execution",
          metrics_path() + "/" + output_p->metrics.task_execution_fname,
          output_p->metrics.collect_interval);

  collector_group()
      .register_collector<metrics::tasks::management_metrics_collector>(
          "tasks::management",
          metrics_path() + "/" + output_p->metrics.task_management_fname,
          output_p->metrics.collect_interval);

  collector_group().register_collector<metrics::cache_metrics_collector>(
      "cache",
      metrics_path() + "/" + output_p->metrics.cache_fname,
      output_p->metrics.collect_interval);
  collector_group().reset_all();
} /* metric_collecting_init() */

/*
 * Work around argos' REGISTER_LOOP_FUNCTIONS() macro which does not support
 * namespaces, so if you have two classes of the same name in two different
 * namespaces, the macro will create the same class definition, giving a linker
 * error.
 */
using namespace argos;
typedef foraging_loop_functions depth1_foraging_loop_functions;
REGISTER_LOOP_FUNCTIONS(depth1_foraging_loop_functions,
                        "depth1_foraging_loop_functions");

NS_END(depth1, support, fordyca);
