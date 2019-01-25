/**
 * @file depth2_loop_functions.cpp
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
/*
 * This is needed because without it boost instantiates static assertions that
 * verify that every possible handler<controller> instantiation is valid, which
 * includes checking for depth1 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT
#include "fordyca/support/depth2/depth2_loop_functions.hpp"

#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/ogrp_mdpo_controller.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/oracle_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/support/block_dist/base_distributor.hpp"
#include "fordyca/support/depth2/depth2_metrics_aggregator.hpp"
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "fordyca/support/depth2/robot_arena_interactor.hpp"
#include "fordyca/support/tasking_oracle.hpp"

#include "rcppsw/swarm/convergence/convergence_params.hpp"
#include "rcppsw/task_allocation/bi_tdgraph.hpp"
#include "rcppsw/task_allocation/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);
using ds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth2_loop_functions::depth2_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.depth2"),
      m_metrics_agg(nullptr),
      m_cache_manager(nullptr),
      m_interactors(nullptr) {}

depth2_loop_functions::~depth2_loop_functions(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void depth2_loop_functions::Init(ticpp::Element& node) {
  depth1::depth1_loop_functions::Init(node);

  ndc_push();
  ER_INFO("Initializing...");

  /* initialize stat collecting */
  auto* arenap = params()->parse_results<params::arena::arena_map_params>();

  params::output_params output =
      *params()->parse_results<const struct params::output_params>();
  auto* conv = params()->parse_results<rswc::convergence_params>();
  output.metrics.arena_grid = arenap->grid;
  m_metrics_agg = rcppsw::make_unique<depth2_metrics_aggregator>(
      &output.metrics, conv, output_root());

  /* initialize cache handling */
  auto* cachep = params()->parse_results<params::caches::caches_params>();
  cache_handling_init(cachep);

  /* intitialize robot interactions with environment */
  m_interactors = rcppsw::make_unique<interactor_map>();
  m_interactors->emplace(typeid(controller::depth2::grp_dpo_controller),
                         grp_dpo_itype(arena_map(),
                                       m_metrics_agg.get(),
                                       floor(),
                                       tv_controller(),
                                       m_cache_manager.get()));
  m_interactors->emplace(typeid(controller::depth2::grp_mdpo_controller),
                         grp_mdpo_itype(arena_map(),
                                        m_metrics_agg.get(),
                                        floor(),
                                        tv_controller(),
                                        m_cache_manager.get()));

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto& controller = dynamic_cast<controller::depth2::grp_mdpo_controller&>(
        robot.GetControllableEntity().GetController());
    controller_configure(controller);
  } /* for(&entity..) */
  ER_INFO("Initialization finished");
  ndc_pop();
}

void depth2_loop_functions::pre_step_iter(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::depth2::grp_mdpo_controller*>(
      &robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  m_metrics_agg->collect_from_controller(controller);
  controller->free_pickup_event(false);
  controller->free_drop_event(false);

  /* send the robot its view of the world: what it sees and where it is */
  loop_utils::set_robot_pos<decltype(*controller)>(robot);
  ER_ASSERT(std::fmod(controller->los_dim(), arena_map()->grid_resolution()) <=
                std::numeric_limits<double>::epsilon(),
            "LOS dimension (%f) not an even multiple of grid resolution (%f)",
            controller->los_dim(),
            arena_map()->grid_resolution());
  uint los_grid_size = controller->los_dim() / arena_map()->grid_resolution();

  loop_utils::set_robot_los<decltype(*controller)>(robot,
                                                   los_grid_size,
                                                   *arena_map());
  set_robot_tick<decltype(*controller)>(robot);

  /* update arena map metrics with robot position */
  auto coord =
      rmath::dvec2uvec(controller->position(), arena_map()->grid_resolution());
  arena_map()->access<arena_grid::kRobotOccupancy>(coord) = true;

  /*
   * The MAGIC of boost so that we can avoid a series of if()/else if() for each
   * of the types of controllers in depth0 for robot-arena interactions.
   *
   * We use the runtime type of the controller we have to index into a map
   * containing a boost::variant of robot_arena_interactor<T>'s. The variant we
   * access in the map will be one with the active type being
   * robot_arena_interactor<runtime-type-of-current-controller>.
   *
   * We then just use a simple visitor to perform all robot-arena interactions.
   *
   * If said interaction results in a block being dropped in a new cache, then
   * we need to re-run dynamic cache creation.
   *
   */
  if (boost::apply_visitor(controller_interactor_mapper(
                               controller, GetSpace().GetSimulationClock()),
                           m_interactors->at(controller->type_index()))) {
    if (cache_creation_handle(true)) {
    } else {
      ER_WARN("Unable to create cache after block drop in new cache");
    }
  }
} /* pre_step_iter() */

void depth2_loop_functions::controller_configure(controller::base_controller& c) {
  /*
   * If NULL, then visualization has been disabled.
   */
  auto& greedy = dynamic_cast<controller::depth2::grp_mdpo_controller&>(c);
  auto* vparams = params()->parse_results<struct params::visualization_params>();
  if (nullptr != vparams) {
    greedy.display_task(vparams->robot_task);
  }

  auto* oraclep = params()->parse_results<params::oracle_params>();
  if (oraclep->enabled) {
    auto& oracular = dynamic_cast<controller::depth2::ogrp_mdpo_controller&>(c);
    oracular.executive()->task_finish_notify(
        std::bind(&tasking_oracle::task_finish_cb,
                  tasking_oracle(),
                  std::placeholders::_1));
    oracular.executive()->task_abort_notify(
        std::bind(&tasking_oracle::task_abort_cb,
                  tasking_oracle(),
                  std::placeholders::_1));
    oracular.tasking_oracle(tasking_oracle());
  }
  greedy.executive()->task_finish_notify(
      std::bind(&depth2_metrics_aggregator::task_finish_or_abort_cb,
                m_metrics_agg.get(),
                std::placeholders::_1));
  greedy.executive()->task_abort_notify(
      std::bind(&depth2_metrics_aggregator::task_finish_or_abort_cb,
                m_metrics_agg.get(),
                std::placeholders::_1));
  greedy.executive()->task_alloc_notify(
      std::bind(&depth2_metrics_aggregator::task_alloc_cb,
                m_metrics_agg.get(),
                std::placeholders::_1,
                std::placeholders::_2));
} /* controller_configure() */

void depth2_loop_functions::cache_handling_init(
    const struct params::caches::caches_params* const cachep) {
  m_cache_manager =
      rcppsw::make_unique<dynamic_cache_manager>(cachep,
                                                 &arena_map()->decoratee());

  cache_creation_handle(false);
} /* cache_handlng_init() */

argos::CColor depth2_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  rmath::vector2d tmp(plane_pos.GetX(), plane_pos.GetY());
  if (arena_map()->nest().contains_point(tmp)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }
  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks rendering inside of caches.
   */
  for (auto& cache : arena_map()->caches()) {
    if (cache->contains_point(tmp)) {
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
    if (block->contains_point(tmp)) {
      return argos::CColor::BLACK;
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void depth2_loop_functions::PreStep() {
  ndc_push();
  base_loop_functions::PreStep();

  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    pre_step_iter(robot);
  } /* for(&entity..) */

  /* handle cache removal */
  if (arena_map()->caches_removed() > 0) {
    m_cache_manager->cache_depleted();
    floor()->SetChanged();
    arena_map()->caches_removed_reset();
  }

  pre_step_final();
  ndc_pop();
} /* PreStep() */

void depth2_loop_functions::Reset(void) {
  ndc_push();
  m_metrics_agg->reset_all();
  cache_creation_handle(false);
  ndc_pop();
}

bool depth2_loop_functions::cache_creation_handle(bool on_drop) {
  auto* cachep = params()->parse_results<params::caches::caches_params>();
  /*
   * If dynamic cache creation is configured to occur only upon a robot dropping
   * a block, then we do not perform cache creation unless that event occurred.
   */
  if (cachep->dynamic.robot_drop_only && !on_drop) {
    ER_INFO("Not performing dynamic cache creation: no robot block drop");
    return false;
  }
  auto ret =
      m_cache_manager->create(arena_map()->caches(),
                              arena_map()->block_distributor()->block_clusters(),
                              arena_map()->blocks());
  if (ret.status) {
    arena_map()->caches_add(ret.caches);
    floor()->SetChanged();
  }
  return ret.status;
} /* cache_creation_handle() */

void depth2_loop_functions::pre_step_final(void) {
  /* collect metrics from/about caches */
  for (auto& c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */

  m_metrics_agg->collect_from_cache_manager(m_cache_manager.get());
  m_cache_manager->reset_metrics();

  /* collect metrics about the arena  */
  m_metrics_agg->collect_from_arena(arena_map());
  m_metrics_agg->collect_from_loop(this);

  m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock());
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();
} /* pre_step_final() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
REGISTER_LOOP_FUNCTIONS(depth2_loop_functions, "depth2_loop_functions");
#pragma clang diagnostic pop
NS_END(depth2, support, fordyca);
