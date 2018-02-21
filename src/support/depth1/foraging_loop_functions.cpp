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
#include "fordyca/metrics/fsm/depth1_metrics_collector.hpp"
#include "fordyca/metrics/fsm/distance_metrics_collector.hpp"
#include "fordyca/metrics/fsm/stateful_metrics_collector.hpp"
#include "fordyca/metrics/fsm/stateless_metrics_collector.hpp"
#include "fordyca/metrics/tasks/execution_metrics_collector.hpp"
#include "fordyca/metrics/tasks/management_metrics_collector.hpp"
#include "rcppsw/metrics/tasks/execution_metrics.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/loop_functions_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_loop_functions::foraging_loop_functions(void)
    : m_depth1_collector(),
      m_task_execution_collector(),
      m_task_management_collector(),
      m_cache_collector(),
      m_cache_penalty_handler() {}

foraging_loop_functions::~foraging_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(argos::TConfigurationNode& node) {
  depth0::stateful_foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth1_foraging loop functions");
  params::loop_function_repository repo;

  repo.parse_all(node);

  /* initialize cache handling and create initial cache */
  cache_handling_init(static_cast<const struct params::arena_map_params*>(
      repo.get_params("arena_map")));

  /* initialize stat collecting */
  metric_collecting_init(static_cast<const struct params::output_params*>(
      repo.get_params("output")));

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = dynamic_cast<controller::depth1::foraging_controller&>(
        robot.GetControllableEntity().GetController());
    auto* l_params = static_cast<const struct params::loop_functions_params*>(
        repo.get_params("loop_functions"));

    controller.display_task(l_params->display_robot_task);
  } /* for(&entity..) */
  ER_NOM("depth1_foraging loop functions initialization finished");
}

void foraging_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
  auto& controller = dynamic_cast<controller::depth1::foraging_controller&>(
      robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  distance_collector()->collect(
      static_cast<metrics::fsm::distance_metrics&>(controller));
  if (nullptr != controller.current_task()) {
    stateless_collector()->collect(
        static_cast<metrics::fsm::stateless_metrics&>(*controller.current_task()));
    stateful_collector()->collect(
        static_cast<metrics::fsm::stateful_metrics&>(*controller.current_task()));
    m_depth1_collector->collect(
        static_cast<metrics::fsm::depth1_metrics&>(*controller.current_task()));
    m_task_execution_collector->collect(
        static_cast<rcppsw::metrics::tasks::execution_metrics&>(*controller.current_task()));
    m_task_management_collector->collect(
        static_cast<rcppsw::metrics::tasks::management_metrics&>(controller));
  }

  /* send the robot its view of the world: what it sees and where it is */
  utils::set_robot_pos<decltype(controller)>(robot);
  set_robot_los<decltype(controller)>(robot, *map());
  set_robot_tick<decltype(controller)>(robot);

  /* now watch it react */
  handle_arena_interactions(robot);
} /* pre_step_iter() */

void foraging_loop_functions::handle_arena_interactions(
    argos::CFootBotEntity& robot) {
  auto& controller = dynamic_cast<controller::depth1::foraging_controller&>(
      robot.GetControllableEntity().GetController());

  if (handle_task_abort<decltype(controller)>(robot)) {
    return;
  }
  if (controller.is_carrying_block()) {
    handle_nest_block_drop<controller::depth1::foraging_controller>(
        robot, *map(), *block_collector());
    if (m_cache_penalty_handler->is_serving_penalty<decltype(controller)>(
            robot)) {
      if (m_cache_penalty_handler->penalty_satisfied<decltype(controller)>(
              robot, GetSpace().GetSimulationClock())) {
        finish_cache_block_drop<controller::depth1::foraging_controller>(robot);
      }
    } else {
      m_cache_penalty_handler->penalty_init<decltype(controller)>(
          robot, GetSpace().GetSimulationClock());
    }
  } else { /* The foot-bot has no block item */
    handle_free_block_pickup<controller::depth1::foraging_controller>(robot,
                                                                      *map());

    if (m_cache_penalty_handler->is_serving_penalty<decltype(controller)>(
            robot)) {
      if (m_cache_penalty_handler->penalty_satisfied<decltype(controller)>(
              robot, GetSpace().GetSimulationClock())) {
        finish_cached_block_pickup<controller::depth1::foraging_controller>(
            robot);
      }
    } else {
      m_cache_penalty_handler->penalty_init<decltype(controller)>(
          robot, GetSpace().GetSimulationClock());
    }
  }
} /* handle_arena_interactions() */

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
  for (auto& cache : map()->caches()) {
    if (cache.contains_point(plane_pos)) {
      return cache.color();
    }
  } /* for(&cache..) */

  for (auto& block : map()->blocks()) {
    if (block.contains_point(plane_pos)) {
      return block.color();
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void foraging_loop_functions::PreStep() {
  /* Get metrics from caches */
  for (auto &c : map()->caches()) {
    m_cache_collector->collect(c);
    c.reset_metrics();
  } /* for(&c..) */

  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  pre_step_final();
} /* PreStep() */

void foraging_loop_functions::Reset() {
  depth0::stateful_foraging_loop_functions::Reset();
  m_depth1_collector->reset();
  m_task_execution_collector->reset();
  m_task_management_collector->reset();
  map()->static_cache_create();
}

void foraging_loop_functions::Destroy() {
  depth0::stateful_foraging_loop_functions::Destroy();
  m_depth1_collector->finalize();
  m_task_execution_collector->finalize();
  m_task_management_collector->finalize();
}

void foraging_loop_functions::pre_step_final(void) {
  depth0::stateful_foraging_loop_functions::pre_step_final();

  /*
   * The cache is recreated with a probability that depends on the relative
   * ratio between the # foragers and the # collectors. If there are more
   * foragers than collectors, then the cache will be recreated very quickly. If
   * there are more collectors than foragers, then it will probably not be
   * recreated immediately. And if there are no foragers, there is no chance
   * that the cache could be recreated (trying to emulate depth2 behavior here).
   */
  if (map()->has_static_cache() && map()->caches().empty()) {
    int n_foragers = m_task_execution_collector->n_foragers();
    int n_collectors = m_task_execution_collector->n_collectors();
    math::cache_respawn_probability p(mc_cache_respawn_scale_factor);
    if (p.calc(n_foragers, n_collectors) >=
        static_cast<double>(random()) / RAND_MAX) {
      map()->static_cache_create();
      representation::cell2D& cell =
          map()->access(map()->caches()[0].discrete_loc());
      ER_ASSERT(map()->caches()[0].n_blocks() == cell.block_count(),
                "FATAL: Cache/cell disagree on # of blocks: cache=%u/cell=%zu",
                map()->caches()[0].n_blocks(),
                cell.block_count());
      floor()->SetChanged();
    }
  }

  if (map()->cache_removed()) {
    floor()->SetChanged();
    map()->cache_removed(false);
  }

  m_depth1_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_depth1_collector->timestep_reset();
  m_depth1_collector->interval_reset();
  m_depth1_collector->timestep_inc();

  m_task_execution_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_task_execution_collector->timestep_reset();
  m_task_execution_collector->interval_reset();
  m_task_execution_collector->timestep_inc();
  m_task_management_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_task_management_collector->timestep_reset();
  m_task_management_collector->interval_reset();
  m_task_management_collector->timestep_inc();
  m_cache_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_cache_collector->timestep_reset();
  m_cache_collector->interval_reset();
  m_cache_collector->timestep_inc();
} /* pre_step_final() */

__const bool foraging_loop_functions::block_drop_overlap_with_cache(
    const representation::block* block,
    const representation::arena_cache& cache,
    const argos::CVector2& drop_loc) {
  return (cache.contains_point(drop_loc + argos::CVector2(block->xsize(), 0)) ||
          cache.contains_point(drop_loc - argos::CVector2(block->xsize(), 0)) ||
          cache.contains_point(drop_loc + argos::CVector2(0, block->ysize())) ||
          cache.contains_point(drop_loc - argos::CVector2(0, block->ysize())));
} /* block_drop_overlap_with_cache() */

__pure bool foraging_loop_functions::block_drop_overlap_with_nest(
    const representation::block* block,
    const argos::CVector2& drop_loc) {
  return (nest_xrange().WithinMinBoundIncludedMaxBoundIncluded(
              drop_loc.GetX() + block->xsize()) ||
          nest_xrange().WithinMinBoundIncludedMaxBoundIncluded(
              drop_loc.GetX() - block->xsize()) ||
          nest_yrange().WithinMinBoundIncludedMaxBoundIncluded(
              drop_loc.GetY() + block->ysize()) ||
          nest_yrange().WithinMinBoundIncludedMaxBoundIncluded(drop_loc.GetY() -
                                                               block->ysize()));
} /* block_drop_overlap_with_nest() */

__pure bool foraging_loop_functions::block_drop_near_arena_boundary(
    const representation::block* block,
    const argos::CVector2& drop_loc) {
  return (drop_loc.GetX() <= block->xsize() * 2  ||
          drop_loc.GetX() >= map()->xrsize() - block->xsize() * 2  ||
          drop_loc.GetY() <= block->ysize() * 2 ||
          drop_loc.GetY() >= map()->yrsize() - block->ysize() * 2);
} /* block_drop_overlap_with_nest() */

void foraging_loop_functions::cache_handling_init(
    const struct params::arena_map_params* arenap) {
  /*
   * Regardless of how many foragers/etc there are, always create an
   * initial cache.
   */
  if (arenap->cache.create_static) {
    map()->static_cache_create();
  }

  m_cache_penalty_handler = std::make_shared<cache_penalty_handler>(
      rcppsw::er::g_server, map(), arenap->cache.usage_penalty);
  mc_cache_respawn_scale_factor = arenap->cache.static_respawn_scale_factor;
} /* cache_handling_init() */

void foraging_loop_functions::metric_collecting_init(
    const struct params::output_params* output_p) {
  m_depth1_collector =
      rcppsw::make_unique<metrics::fsm::depth1_metrics_collector>(
          metrics_path() + "/" + output_p->metrics.depth1_fname,
          output_p->metrics.collect_cum,
          output_p->metrics.collect_interval);
  m_depth1_collector->reset();

  m_task_execution_collector =
      rcppsw::make_unique<metrics::tasks::execution_metrics_collector>(
      metrics_path() + "/" + output_p->metrics.task_execution_fname,
      output_p->metrics.collect_cum,
      output_p->metrics.collect_interval);
  m_task_execution_collector->reset();

  m_task_management_collector =
      rcppsw::make_unique<metrics::tasks::management_metrics_collector>(
      metrics_path() + "/" + output_p->metrics.task_management_fname,
      output_p->metrics.collect_cum,
      output_p->metrics.collect_interval);
  m_task_management_collector->reset();

  m_cache_collector =
      rcppsw::make_unique<metrics::cache_metrics_collector>(
          metrics_path() + "/" + output_p->metrics.cache_fname,
          output_p->metrics.collect_cum,
          output_p->metrics.collect_interval);
  m_cache_collector->reset();
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
