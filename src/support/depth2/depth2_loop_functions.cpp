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
 * includes checking for depth2 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT
#include "fordyca/support/depth2/depth2_loop_functions.hpp"
#include <boost/mpl/for_each.hpp>

#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_odpo_controller.hpp"
#include "fordyca/controller/depth2/grp_omdpo_controller.hpp"
#include "fordyca/params/arena/arena_map_params.hpp"
#include "fordyca/params/output_params.hpp"
#include "fordyca/params/visualization_params.hpp"
#include "fordyca/support/block_dist/base_distributor.hpp"
#include "fordyca/support/depth2/depth2_metrics_aggregator.hpp"
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "fordyca/support/depth2/robot_arena_interactor.hpp"
#include "fordyca/support/depth2/robot_configurer.hpp"
#include "fordyca/support/depth2/robot_configurer_adaptor.hpp"
#include "fordyca/support/oracle/oracle_manager.hpp"
#include "fordyca/support/robot_interactor_adaptor.hpp"
#include "fordyca/support/robot_los_updater_adaptor.hpp"
#include "fordyca/support/robot_metric_extractor_adaptor.hpp"
#include "fordyca/support/robot_task_extractor_adaptor.hpp"

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/swarm/convergence/convergence_calculator.hpp"
#include "rcppsw/ta/bi_tdgraph.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth2);
using ds::arena_grid;

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

/**
 * @struct functor_maps_initializer
 * @ingroup fordyca support depth2 detail
 *
 * Convenience class containing initialization for all of the typeid ->
 * boost::variant maps for all controller types that are used throughout
 * initialization and simulation.
 */
struct functor_maps_initializer : public boost::static_visitor<void> {
  functor_maps_initializer(configurer_map_type* const cmap,
                           depth2_loop_functions* const lf_in)

      : lf(lf_in), config_map(cmap) {}
  template <typename T>
  void operator()(const T& controller) const {
    lf->m_interactor_map->emplace(
        typeid(controller),
        robot_arena_interactor<T>(lf->arena_map(),
                                  lf->m_metrics_agg.get(),
                                  lf->floor(),
                                  lf->tv_manager(),
                                  lf->m_cache_manager.get()));
    lf->m_metric_extractor_map->emplace(
        typeid(controller),
        robot_metric_extractor<depth2_metrics_aggregator, T>(
            lf->m_metrics_agg.get()));
    config_map->emplace(
        typeid(controller),
        robot_configurer<T>(
            lf->params()->parse_results<params::visualization_params>(),
            lf->oracle_manager()->entities_oracle(),
            lf->oracle_manager()->tasking_oracle(),
            lf->m_metrics_agg.get()));
    lf->m_los_update_map->emplace(typeid(controller),
                                  robot_los_updater<T>(lf->arena_map()));
  }

  /* clang-format off */
  depth2_loop_functions * const      lf;
  configurer_map_type* const config_map;
  /* clang-format on */
};

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth2_loop_functions::depth2_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.depth2"),
      m_metrics_agg(nullptr),
      m_cache_manager(nullptr),
      m_interactor_map(nullptr),
      m_metric_extractor_map(nullptr),
      m_los_update_map(nullptr),
      m_task_extractor_map(nullptr) {}

depth2_loop_functions::~depth2_loop_functions(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void depth2_loop_functions::Init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void depth2_loop_functions::shared_init(ticpp::Element& node) {
  depth1_loop_functions::shared_init(node);

  /* initialize stat collecting */
  auto* arenap = params()->parse_results<params::arena::arena_map_params>();

  params::output_params output =
      *params()->parse_results<const struct params::output_params>();
  output.metrics.arena_grid = arenap->grid;
  m_metrics_agg = rcppsw::make_unique<depth2_metrics_aggregator>(&output.metrics,
                                                                 output_root());
} /* shared_init() */

void depth2_loop_functions::private_init(void) {
  /* initialize cache handling */
  auto* cachep = params()->parse_results<params::caches::caches_params>();
  cache_handling_init(cachep);

  /*
   * Initialize convergence calculations to include task distribution (not
   * included by default).
   */
  conv_calculator()->task_dist_init(std::bind(
      &depth2_loop_functions::robot_tasks_extract, this, std::placeholders::_1));

  /*
   * Intitialize robot interactions with environment wth various functors/type
   * maps.
   */
  m_interactor_map = rcppsw::make_unique<interactor_map_type>();
  m_metric_extractor_map = rcppsw::make_unique<metric_extractor_map_type>();
  m_los_update_map = rcppsw::make_unique<los_updater_map_type>();
  m_task_extractor_map = rcppsw::make_unique<task_extractor_map_type>();

  /* only needed for initialization, so not a member */
  auto config_map = detail::configurer_map_type();

  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::depth2::typelist>(f_initializer);

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto base = dynamic_cast<controller::base_controller*>(
        &robot.GetControllableEntity().GetController());
    boost::apply_visitor(detail::robot_configurer_adaptor(base),
                         config_map.at(base->type_index()));
  } /* for(&entity..) */
} /* private_init() */

void depth2_loop_functions::cache_handling_init(
    const struct params::caches::caches_params* const cachep) {
  m_cache_manager =
      rcppsw::make_unique<dynamic_cache_manager>(cachep,
                                                 &arena_map()->decoratee());

  cache_creation_handle(false);
} /* cache_handlng_init() */

/*******************************************************************************
 * Convergence Calculations Callbacks
 ******************************************************************************/
std::vector<int> depth2_loop_functions::robot_tasks_extract(uint) const {
  std::vector<int> v;
  auto& robots = GetSpace().GetEntitiesByType("foot-bot");

  for (auto& entity_pair : robots) {
    auto* robot = argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto base = dynamic_cast<controller::base_controller*>(
        &robot->GetControllableEntity().GetController());
    v.push_back(
        boost::apply_visitor(robot_task_extractor_adaptor(base),
                             m_task_extractor_map->at(base->type_index())));
  } /* for(&entity..) */
  return v;
} /* robot_tasks_extract() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void depth2_loop_functions::PreStep() {
  ndc_push();

  base_loop_functions::PreStep();

  /* Collect metrics from/about caches */
  for (auto& c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */

  m_metrics_agg->collect_from_cache_manager(m_cache_manager.get());
  m_cache_manager->reset_metrics();

  /* Before processing all robots, update the oracles */
  oracle_manager()->update(arena_map());

  /* Process all robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    robot_timestep_process(robot);
  } /* for(&entity..) */

  /* handle cache removal as a result of robot interactions with arena */
  if (m_cache_manager->caches_depleted() > 0) {
    floor()->SetChanged();
  }
  /* collect metrics from non-robot sources */
  m_metrics_agg->collect_from_arena(arena_map());
  m_metrics_agg->collect_from_loop(this);

  /* Not a clean way to do this in the convergence metrics collector... */
  if (m_metrics_agg->metrics_write_all(GetSpace().GetSimulationClock())) {
    conv_calculator()->reset_metrics();
  }
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->timestep_reset_all();
  m_metrics_agg->interval_reset_all();

  ndc_pop();
} /* PreStep() */

void depth2_loop_functions::robot_timestep_process(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::base_controller*>(
      &robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  auto madaptor =
      robot_metric_extractor_adaptor<depth2_metrics_aggregator>(controller);
  boost::apply_visitor(madaptor,
                       m_metric_extractor_map->at(controller->type_index()));
  controller->block_manip_collator()->reset();

  /* Set robot position, time, and send it its new LOS */
  loop_utils::set_robot_pos<decltype(*controller)>(
      robot, arena_map()->grid_resolution());
  loop_utils::set_robot_tick<decltype(*controller)>(
      robot, GetSpace().GetSimulationClock());
  boost::apply_visitor(robot_los_updater_adaptor(controller),
                       m_los_update_map->at(controller->type_index()));

  /* update arena map metrics with robot position */
  arena_map()->access<arena_grid::kRobotOccupancy>(
      controller->discrete_position()) = true;

  /*
   * Watch the robot interact with its environment!
   *
   * If said interaction results in a block being dropped in a new cache, then
   * we need to re-run dynamic cache creation.
   */
  auto iadaptor = robot_interactor_adaptor<robot_arena_interactor, bool>(
      controller, GetSpace().GetSimulationClock());
  bool nc_drop =
      boost::apply_visitor(iadaptor,
                           m_interactor_map->at(controller->type_index()));
  if (nc_drop) {
    if (cache_creation_handle(true)) {
    } else {
      ER_WARN("Unable to create cache after block drop in new cache");
    }
  }
} /* robot_timestep_process() */

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

void depth2_loop_functions::Reset(void) {
  ndc_push();
  base_loop_functions::Reset();
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
  auto created =
      m_cache_manager->create(arena_map()->caches(),
                              arena_map()->block_distributor()->block_clusters(),
                              arena_map()->blocks(),
                              GetSpace().GetSimulationClock());
  if (created) {
    arena_map()->caches_add(*created);
    floor()->SetChanged();
    return true;
  }
  return false;
} /* cache_creation_handle() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
REGISTER_LOOP_FUNCTIONS(depth2_loop_functions, "depth2_loop_functions");
#pragma clang diagnostic pop
NS_END(depth2, support, fordyca);
