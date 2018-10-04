/**
 * @file depth1_loop_functions.cpp
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
#include "fordyca/support/depth1/depth1_loop_functions.hpp"

#include "fordyca/controller/base_controller.hpp"
#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"
#include "fordyca/controller/depth1/oracular_partitioning_controller.hpp"
#include "fordyca/ds/cell2D.hpp"
#include "fordyca/math/cache_respawn_probability.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/oracle_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/support/depth1/depth1_metrics_aggregator.hpp"
#include "fordyca/support/depth1/static_cache_manager.hpp"
#include "fordyca/support/tasking_oracle.hpp"
#include "fordyca/tasks/depth1/existing_cache_interactor.hpp"
#include "rcppsw/metrics/tasks/bifurcating_tab_metrics_collector.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph.hpp"
#include "rcppsw/task_allocation/bifurcating_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth1_loop_functions::depth1_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.depth1"),
      m_tasking_oracle(nullptr),
      m_metrics_agg(nullptr),
      m_cache_manager(nullptr) {}

depth1_loop_functions::~depth1_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth1_loop_functions::Init(ticpp::Element& node) {
  depth0::stateful_loop_functions::Init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* initialize stat collecting */
  auto* arenap = params().parse_results<params::arena::arena_map_params>();
  params::output_params output =
      *params().parse_results<const struct params::output_params>();
  output.metrics.arena_grid = arenap->grid;
  m_metrics_agg = rcppsw::make_unique<depth1_metrics_aggregator>(&output.metrics,
                                                                 output_root());

  /* initialize cache handling and create initial cache */
  cache_handling_init(&arenap->cache);

  /* intitialize robot interactions with environment */
  m_interactor =
      rcppsw::make_unique<interactor>(arena_map(),
                                      m_metrics_agg.get(),
                                      floor(),
                                      &arenap->blocks.manipulation_penalty,
                                      &arenap->cache.usage_penalty);

  /* initialize oracles */
  oracle_init();

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller =
        dynamic_cast<controller::depth1::greedy_partitioning_controller&>(
            robot.GetControllableEntity().GetController());
    controller_configure(controller);
  } /* for(&entity..) */
  ndc_pop();
  ER_INFO("Initialization finished");
}

void depth1_loop_functions::oracle_init(void) {
  auto* oraclep = params().parse_results<params::oracle_params>();
  if (oraclep->tasking_enabled) {
    ER_INFO("Creating oracle");
    argos::CFootBotEntity& robot0 = *argos::any_cast<argos::CFootBotEntity*>(
        GetSpace().GetEntitiesByType("foot-bot").begin()->second);
    const auto& controller0 =
        dynamic_cast<controller::depth1::greedy_partitioning_controller&>(
            robot0.GetControllableEntity().GetController());
    auto* bigraph = dynamic_cast<const ta::bifurcating_tdgraph*>(
        controller0.executive()->graph());
    m_tasking_oracle = std::make_unique<support::tasking_oracle>(bigraph);
  }
} /* oracle_init() */

void depth1_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
  auto& controller =
      dynamic_cast<controller::depth1::greedy_partitioning_controller&>(
          robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  m_metrics_agg->collect_from_controller(&controller);
  controller.free_pickup_event(false);
  controller.free_drop_event(false);

  /* send the robot its view of the world: what it sees and where it is */
  utils::set_robot_pos<decltype(controller)>(robot);
  utils::set_robot_los<decltype(controller)>(robot, *arena_map());
  set_robot_tick<decltype(controller)>(robot);

  /* update arena map metrics with robot position */
  auto coord = math::rcoord_to_dcoord(controller.robot_loc(),
                                      arena_map()->grid_resolution());
  arena_map()->access<arena_grid::kRobotOccupancy>(coord) = true;

  /* Now watch it react to the environment */
  (*m_interactor)(controller, GetSpace().GetSimulationClock());
} /* pre_step_iter() */

void depth1_loop_functions::controller_configure(controller::base_controller& c) {
  /*
   * If NULL, then visualization has been disabled.
   */
  auto& greedy =
      dynamic_cast<controller::depth1::greedy_partitioning_controller&>(c);
  auto* vparams = params().parse_results<struct params::visualization_params>();
  if (nullptr != vparams) {
    greedy.display_task(vparams->robot_task);
  }

  auto* oraclep = params().parse_results<params::oracle_params>();
  auto& oracular =
      dynamic_cast<controller::depth1::oracular_partitioning_controller&>(c);
  if (oraclep->tasking_enabled) {
    oracular.executive()->task_finish_notify(
        std::bind(&tasking_oracle::task_finish_cb,
                  m_tasking_oracle.get(),
                  std::placeholders::_1));
    oracular.executive()->task_abort_notify(
        std::bind(&tasking_oracle::task_abort_cb,
                  m_tasking_oracle.get(),
                  std::placeholders::_1));
    oracular.tasking_oracle(m_tasking_oracle.get());
  }
  greedy.executive()->task_finish_notify(
      std::bind(&depth1_metrics_aggregator::task_finish_or_abort_cb,
                m_metrics_agg.get(),
                std::placeholders::_1));
  greedy.executive()->task_abort_notify(
      std::bind(&depth1_metrics_aggregator::task_finish_or_abort_cb,
                m_metrics_agg.get(),
                std::placeholders::_1));
  greedy.executive()->task_alloc_notify(
      std::bind(&depth1_metrics_aggregator::task_alloc_cb,
                m_metrics_agg.get(),
                std::placeholders::_1,
                std::placeholders::_2));
} /* controller_configure() */

argos::CColor depth1_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  if (arena_map()->nest().contains_point(plane_pos)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }
  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks renderin inside of caches.
   */
  for (auto& cache : arena_map()->caches()) {
    if (cache->contains_point(plane_pos)) {
      return argos::CColor(cache->color().red(),
                           cache->color().green(),
                           cache->color().blue());
    }
  } /* for(&cache..) */

  for (auto& block : arena_map()->blocks()) {
    /*
     * Even though each block type has a unique color, the only distinction
     * that robots can make to determine if they are on a block or not is
     * between shades of black/white. So, all blocks must appear as black, even
     * when they are not actually (when blocks are picked up their correct color
     * is shown through visualization).
     */
    if (block->contains_point(plane_pos)) {
      return argos::CColor::BLACK;
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void depth1_loop_functions::PreStep() {
  ndc_push();
  base_loop_functions::PreStep();
  /* Get metrics from caches */
  for (auto& c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */
  m_metrics_agg->collect_from_cache_manager(m_cache_manager.get());
  m_cache_manager->reset_metrics();

  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */
  m_metrics_agg->collect_from_arena(arena_map());
  m_metrics_agg->collect_from_loop(this);
  pre_step_final();
  ndc_pop();
} /* PreStep() */

void depth1_loop_functions::Reset() {
  m_metrics_agg->reset_all();
  /* return value ignored (for now...) */
  auto pair = m_cache_manager->create(arena_map()->blocks());
  arena_map()->caches_add(pair.second);
}

void depth1_loop_functions::pre_step_final(void) {
  /*
   * The cache is recreated with a probability that depends on the relative
   * ratio between the # foragers and the # collectors. If there are more
   * foragers than collectors, then the cache will be recreated very quickly. If
   * there are more collectors than foragpers, then it will probably not be
   * recreated immediately. And if there are no foragers, there is no chance
   * that the cache could be recreated (trying to emulate depth2 behavior here).
   */
  if (arena_map()->caches().empty()) {
    auto& collector =
        static_cast<rcppsw::metrics::tasks::bifurcating_tab_metrics_collector&>(
            *(*m_metrics_agg)["tasks::generalist_tab"]);
    uint n_harvesters = collector.int_subtask1_count();
    uint n_collectors = collector.int_subtask2_count();
    math::cache_respawn_probability p(mc_cache_respawn_scale_factor);
    if (p.calc(n_harvesters, n_collectors) >=
        static_cast<double>(std::rand()) / RAND_MAX) {
      auto pair = m_cache_manager->create(arena_map()->blocks());

      if (pair.first) {
        arena_map()->caches_add(pair.second);
        __rcsw_unused ds::cell2D& cell = arena_map()->access<arena_grid::kCell>(
            arena_map()->caches()[0]->discrete_loc());
        ER_ASSERT(arena_map()->caches()[0]->n_blocks() == cell.block_count(),
                  "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
                  arena_map()->caches()[0]->n_blocks(),
                  cell.block_count());
        m_cache_manager->cache_created();
        floor()->SetChanged();
      } else {
        ER_WARN("Unable to (re)-create static cache--not enough free blocks?");
      }
    }
  }
  if (arena_map()->caches_removed() > 0) {
    m_cache_manager->cache_depleted();
    floor()->SetChanged();
    arena_map()->caches_removed_reset();
  }

  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
} /* pre_step_final() */

void depth1_loop_functions::cache_handling_init(
    const struct params::arena::cache_params* cachep) {
  /*
   * Regardless of how many foragers/etc there are, always create an
   * initial cache.
   */
  m_cache_loc = argos::CVector2(arena_map()->xrsize() +
                                    arena_map()->nest().real_loc().GetX() / 2.0,
                                arena_map()->nest().real_loc().GetY());

  m_cache_manager = rcppsw::make_unique<static_cache_manager>(cachep,
                                                              &arena_map()->decoratee(),
                                                              m_cache_loc);

  /* return value ignored at this level (for now...) */
  auto pair = m_cache_manager->create(arena_map()->blocks());
  arena_map()->caches_add(pair.second);
} /* cache_handling_init() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
REGISTER_LOOP_FUNCTIONS(depth1_loop_functions, "depth1_loop_functions");
#pragma clang diagnostic pop
NS_END(depth1, support, fordyca);
