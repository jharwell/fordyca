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
#include "fordyca/events/free_block_drop.hpp"
#include "fordyca/expressions/cache_respawn_probability.hpp"
#include "fordyca/metrics/collectors/robot_metrics/depth1_metrics_collector.hpp"
#include "fordyca/metrics/collectors/robot_metrics/distance_metrics_collector.hpp"
#include "fordyca/metrics/collectors/robot_metrics/stateful_metrics_collector.hpp"
#include "fordyca/metrics/collectors/robot_metrics/stateless_metrics_collector.hpp"
#include "fordyca/metrics/collectors/task_collector.hpp"
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/loop_functions_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/representation/cell2D.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
namespace rmetrics = metrics::collectible_metrics::robot_metrics;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
foraging_loop_functions::foraging_loop_functions(void)
    : mc_cache_penalty(),
      mc_cache_respawn_scale_factor(),
      m_depth1_collector(),
      m_task_collector(),
      m_penalty_list() {}

foraging_loop_functions::~foraging_loop_functions(void) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_loop_functions::Init(argos::TConfigurationNode &node) {
  depth0::stateful_foraging_loop_functions::Init(node);

  ER_NOM("Initializing depth1_foraging loop functions");
  params::loop_function_repository repo;

  repo.parse_all(node);

  const struct params::arena_map_params *arenap =
      static_cast<const struct params::arena_map_params *>(
          repo.get_params("arena_map"));

  /*
   * Regardless of how many foragers/collectors/etc there are, always create an
   * initial cache
   */
  if (arenap->cache.create_static) {
    map()->static_cache_create();
  }

  mc_cache_penalty = arenap->cache.usage_penalty;
  mc_cache_respawn_scale_factor = arenap->cache.static_respawn_scale_factor;

  /* initialize stat collecting */
  const params::output_params *p_output =
      static_cast<const struct params::output_params *>(
          repo.get_params("output"));

  m_depth1_collector.reset(new robot_collectors::depth1_metrics_collector(
      metrics_path() + "/" + p_output->metrics.depth1_fname,
      p_output->metrics.collect_cum,
      p_output->metrics.collect_interval));
  m_depth1_collector->reset();
  m_task_collector.reset(new metrics::collectors::task_collector(
      metrics_path() + "/" + p_output->metrics.task_fname,
      p_output->metrics.collect_cum,
      p_output->metrics.collect_interval));
  m_task_collector->reset();

  ER_NOM("depth1_foraging loop functions initialization finished");
}

void foraging_loop_functions::pre_step_iter(argos::CFootBotEntity &robot) {
  controller::depth1::foraging_controller &controller =
      dynamic_cast<controller::depth1::foraging_controller &>(
          robot.GetControllableEntity().GetController());

  /*
   * If a robot aborted its task and was carrying a block it needs to drop it,
   * in addition to updating its own internal state, so that the block is not
   * left dangling and unusable for the rest of the simulation.
   *
   * Also, if the robot happens to abort its task while serving the cache
   * penalty, then it needs to be removed from the penalty list to keep things
   * consistent and avoid assertion failures later.
   */
  if (controller.task_aborted()) {
    if (controller.is_carrying_block()) {
      representation::discrete_coord d =
          representation::real_to_discrete_coord(controller.robot_loc(),
                                                 map()->grid_resolution());
      events::free_block_drop drop_op(rcppsw::er::g_server,
                                      controller.block(),
                                      d.first,
                                      d.second,
                                      map()->grid_resolution());

      controller.block(nullptr);
      map()->accept(drop_op);
      floor()->SetChanged();
    }
    auto it = std::find_if(m_penalty_list.begin(),
                           m_penalty_list.end(),
                           [&](const cache_usage_penalty *p) {
                             return p->controller() == &controller;
                           });
    if (it != m_penalty_list.end()) {
      m_penalty_list.remove(*it);
    }
    ER_ASSERT(
        !robot_serving_cache_penalty<controller::depth1::foraging_controller>(
            robot),
        "FATAL: Multiple instances of same controller serving cache penalty");
  }

  /* get stats from this robot before its state changes */
  stateless_collector()->collect(
      static_cast<rmetrics::stateless_metrics &>(controller));
  stateful_collector()->collect(
      static_cast<rmetrics::stateful_metrics &>(controller));
  distance_collector()->collect(
      static_cast<rmetrics::distance_metrics &>(controller));
  m_depth1_collector->collect(
      static_cast<rmetrics::depth1_metrics &>(controller));
  m_task_collector->collect(
      static_cast<metrics::collectible_metrics::task_metrics &>(controller));

  utils::set_robot_pos<controller::depth1::foraging_controller>(robot);
  utils::set_robot_los<controller::depth1::foraging_controller>(robot, *map());
  set_robot_tick<controller::depth1::foraging_controller>(robot);

  if (controller.task_aborted()) {
    return;
  }
  if (controller.is_carrying_block()) {
    handle_nest_block_drop<controller::depth1::foraging_controller>(
        robot, *map(), *block_collector());
    if (robot_serving_cache_penalty<controller::depth1::foraging_controller>(
            robot)) {
      if (cache_usage_penalty_satisfied<controller::depth1::foraging_controller>(
              robot)) {
        handle_cache_block_drop<controller::depth1::foraging_controller>(robot);
      }
    } else {
      init_cache_usage_penalty<controller::depth1::foraging_controller>(robot);
    }
  } else { /* The foot-bot has no block item */
    handle_free_block_pickup<controller::depth1::foraging_controller>(robot,
                                                                      *map());

    if (robot_serving_cache_penalty<controller::depth1::foraging_controller>(
            robot)) {
      if (cache_usage_penalty_satisfied<controller::depth1::foraging_controller>(
              robot)) {
        finish_cached_block_pickup<controller::depth1::foraging_controller>(
            robot);
      }
    } else {
      init_cache_usage_penalty<controller::depth1::foraging_controller>(robot);
    }
  }
} /* pre_step_iter() */

argos::CColor foraging_loop_functions::GetFloorColor(
    const argos::CVector2 &plane_pos) {
  /* The nest is a light gray */
  if (nest_xrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetX()) &&
      nest_yrange().WithinMinBoundIncludedMaxBoundIncluded(plane_pos.GetY())) {
    return argos::CColor::GRAY70;
  }

  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks renderin inside of caches.
   */
  for (size_t i = 0; i < map()->caches().size(); ++i) {
    if (map()->caches()[i].contains_point(plane_pos)) {
      return argos::CColor::GRAY40;
    }
  } /* for(i..) */

  for (size_t i = 0; i < map()->blocks().size(); ++i) {
    if (map()->blocks()[i].contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(i..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void foraging_loop_functions::PreStep() {
  argos::CSpace::TMapPerType &footbots =
      GetSpace().GetEntitiesByType("foot-bot");

  for (argos::CSpace::TMapPerType::iterator it = footbots.begin();
       it != footbots.end();
       ++it) {
    argos::CFootBotEntity &robot =
        *argos::any_cast<argos::CFootBotEntity *>(it->second);
    pre_step_iter(robot);
  } /* for(it..) */
  pre_step_final();
} /* PreStep() */

void foraging_loop_functions::Reset() {
  depth0::stateful_foraging_loop_functions::Reset();
  m_depth1_collector->reset();
  m_task_collector->reset();
  map()->static_cache_create();
}

void foraging_loop_functions::Destroy() {
  depth0::stateful_foraging_loop_functions::Destroy();
  m_depth1_collector->finalize();
  m_task_collector->finalize();
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
  if (map()->has_static_cache() && 0 == map()->caches().size()) {
    int n_foragers = m_task_collector->n_foragers();
    int n_collectors = m_task_collector->n_collectors();
    expressions::cache_respawn_probability p(mc_cache_respawn_scale_factor);
    if (p.calc(n_foragers, n_collectors) >=
        static_cast<double>(rand()) / RAND_MAX) {
      map()->static_cache_create();
      representation::cell2D &cell =
          map()->access(map()->caches()[0].discrete_loc());
      ER_ASSERT(map()->caches()[0].n_blocks() == cell.block_count(),
                "FATAL: Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
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

  m_task_collector->csv_line_write(GetSpace().GetSimulationClock());
  m_task_collector->timestep_reset();
  m_task_collector->interval_reset();
  m_task_collector->timestep_inc();
} /* pre_step_final() */

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
