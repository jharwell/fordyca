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
/*
 * This is needed because without it boost instantiates static assertions that
 * verify that every possible handler<controller> instantiation is valid, which
 * includes checking for depth1 controllers being valid for new cache drop/cache
 * site drop events. These will not happen in reality (or shouldn't), and if
 * they do it's 100% OK to crash with an exception.
 */
#define BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT
#include "fordyca/support/depth1/depth1_loop_functions.hpp"
#include <boost/mpl/for_each.hpp>

#include "fordyca/config/arena/arena_map_config.hpp"
#include "fordyca/config/oracle/oracle_manager_config.hpp"
#include "fordyca/config/output_config.hpp"
#include "fordyca/config/visualization_config.hpp"
#include "fordyca/controller/depth1/gp_dpo_controller.hpp"
#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"
#include "fordyca/controller/depth1/gp_odpo_controller.hpp"
#include "fordyca/controller/depth1/gp_omdpo_controller.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/support/depth1/depth1_metrics_aggregator.hpp"
#include "fordyca/support/depth1/robot_arena_interactor.hpp"
#include "fordyca/support/depth1/robot_configurer.hpp"
#include "fordyca/support/depth1/robot_configurer_adaptor.hpp"
#include "fordyca/support/depth1/static_cache_manager.hpp"
#include "fordyca/support/oracle/oracle_manager.hpp"
#include "fordyca/support/robot_interactor_adaptor.hpp"
#include "fordyca/support/robot_los_updater_adaptor.hpp"
#include "fordyca/support/robot_metric_extractor_adaptor.hpp"
#include "fordyca/support/robot_task_extractor.hpp"
#include "fordyca/support/robot_task_extractor_adaptor.hpp"
#include "rcppsw/swarm/convergence/convergence_calculator.hpp"
#include "rcppsw/ta/bi_tdgraph.hpp"
#include "rcppsw/ta/bi_tdgraph_executive.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth1);
using ds::arena_grid;

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

template <class ControllerType>
struct d1_subtask_status_extractor
    : boost::static_visitor<std::pair<bool, bool>> {
  using controller_type = ControllerType;
  d1_subtask_status_extractor(void) = default;

  std::pair<bool, bool> operator()(const ControllerType* const c) const {
    auto task = dynamic_cast<const rta::polled_task*>(c->current_task());
    return std::make_pair(
        task->name() == tasks::depth1::foraging_task::kHarvesterName,
        task->name() == tasks::depth1::foraging_task::kCollectorName);
  }
};

/**
 * @struct d1_subtask_status_extractor_adaptor
 * @ingroup fordyca support depth1
 *
 * @brief Calculate the \ref collector, \ref harvester task counts for depth1
 * when a static cache is depleted, for use in determining the static cache
 * respawn probability.
 */
struct d1_subtask_status_extractor_adaptor
    : public boost::static_visitor<std::pair<bool, bool>> {
  explicit d1_subtask_status_extractor_adaptor(
      const controller::base_controller* const c)
      : mc_controller(c) {}

  template <typename ControllerType>
  std::pair<bool, bool> operator()(
      const d1_subtask_status_extractor<ControllerType>& extractor) const {
    auto cast = dynamic_cast<const typename d1_subtask_status_extractor<
        ControllerType>::controller_type*>(mc_controller);
    return extractor(cast);
  }
  const controller::base_controller* const mc_controller;
};

/**
 * @struct functor_maps_initializer
 * @ingroup fordyca support depth1 detail
 *
 * Convenience class containing initialization for all of the typeid ->
 * boost::variant maps for all controller types that are used throughout
 * initialization and simulation.
 */
struct functor_maps_initializer : public boost::static_visitor<void> {
  functor_maps_initializer(configurer_map_type* const cmap,
                           depth1_loop_functions* const lf_in)

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
        robot_metric_extractor<depth1_metrics_aggregator, T>(
            lf->m_metrics_agg.get()));
    config_map->emplace(
        typeid(controller),
        robot_configurer<T, depth1_metrics_aggregator>(
            lf->config()->config_get<config::visualization_config>(),
            lf->oracle_manager()->entities_oracle(),
            lf->oracle_manager()->tasking_oracle(),
            lf->m_metrics_agg.get()));
    lf->m_los_update_map->emplace(typeid(controller),
                                  robot_los_updater<T>(lf->arena_map()));
    lf->m_subtask_status_map->emplace(typeid(controller),
                                      d1_subtask_status_extractor<T>());
  }

  /* clang-format off */
  depth1_loop_functions * const lf;
  configurer_map_type* const    config_map;
  /* clang-format on */
};

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth1_loop_functions::depth1_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.depth1"),
      m_interactor_map(nullptr),
      m_metric_extractor_map(nullptr),
      m_los_update_map(nullptr),
      m_task_extractor_map(nullptr),
      m_subtask_status_map(nullptr),
      m_metrics_agg(nullptr),
      m_cache_manager(nullptr) {}

depth1_loop_functions::~depth1_loop_functions(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void depth1_loop_functions::Init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void depth1_loop_functions::shared_init(ticpp::Element& node) {
  depth0_loop_functions::shared_init(node);

  /* initialize stat collecting */
  auto* arenap = config()->config_get<config::arena::arena_map_config>();
  config::output_config output =
      *config()->config_get<const config::output_config>();
  output.metrics.arena_grid = arenap->grid;
  m_metrics_agg = rcppsw::make_unique<depth1_metrics_aggregator>(&output.metrics,
                                                                 output_root());
  /* initialize tasking oracle */
  oracle_init();
} /* shared_init() */

void depth1_loop_functions::private_init(void) {
  /* initialize cache handling and create initial cache */
  cache_handling_init(config()->config_get<config::caches::caches_config>());

  /*
   * Initialize convergence calculations to include task distribution (not
   * included by default).
   */
  conv_calculator()->task_dist_init(std::bind(
      &depth1_loop_functions::robot_tasks_extract, this, std::placeholders::_1));

  /*
   * Intitialize robot interactions with environment via various functors/type
   * maps.
   */
  m_interactor_map = rcppsw::make_unique<interactor_map_type>();
  m_metric_extractor_map = rcppsw::make_unique<metric_extractor_map_type>();
  m_los_update_map = rcppsw::make_unique<los_updater_map_type>();
  m_task_extractor_map = rcppsw::make_unique<task_extractor_map_type>();
  m_subtask_status_map =
      rcppsw::make_unique<detail::d1_subtask_status_map_type>();

  /* only needed for initialization, so not a member */
  auto config_map = configurer_map_type();

  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::depth1::typelist>(f_initializer);

  /* configure robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto base = dynamic_cast<controller::base_controller*>(
        &robot.GetControllableEntity().GetController());
    boost::apply_visitor(robot_configurer_adaptor(base),
                         config_map.at(base->type_index()));
  } /* for(entity_pair..) */
} /* private_init() */

void depth1_loop_functions::oracle_init(void) {
  auto* oraclep = config()->config_get<config::oracle::oracle_manager_config>();
  if (oraclep->tasking.enabled) {
    argos::CFootBotEntity& robot0 = *argos::any_cast<argos::CFootBotEntity*>(
        GetSpace().GetEntitiesByType("foot-bot").begin()->second);
    const auto& controller0 =
        dynamic_cast<controller::depth1::gp_mdpo_controller&>(
            robot0.GetControllableEntity().GetController());
    auto* bigraph =
        dynamic_cast<const rta::bi_tdgraph*>(controller0.executive()->graph());
    oracle_manager()->tasking_oracle(
        std::make_unique<support::oracle::tasking_oracle>(&oraclep->tasking,
                                                          bigraph));
  }
} /* oracle_init() */

void depth1_loop_functions::cache_handling_init(
    const config::caches::caches_config* cachep) {
  if (nullptr != cachep && cachep->static_.enable) {
    /*
     * Regardless of how many foragers/etc there are, always create an
     * initial cache.
     */
    rmath::vector2d cache_loc = rmath::vector2d(
        (arena_map()->xrsize() + arena_map()->nest().real_loc().x()) / 2.0,
        arena_map()->nest().real_loc().y());

    m_cache_manager = rcppsw::make_unique<static_cache_manager>(
        cachep, &arena_map()->decoratee(), cache_loc);

    if (auto created = m_cache_manager->create(
            arena_map()->blocks(), GetSpace().GetSimulationClock())) {
      arena_map()->caches_add(*created);
      floor()->SetChanged();
    }
  }
} /* cache_handling_init() */

/*******************************************************************************
 * Convergence Calculations Callbacks
 ******************************************************************************/
std::vector<int> depth1_loop_functions::robot_tasks_extract(uint) const {
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
void depth1_loop_functions::PreStep() {
  ndc_push();
  base_loop_functions::PreStep();

  auto& collector = static_cast<metrics::blocks::transport_metrics_collector&>(
      *(*m_metrics_agg)["blocks::transport"]);
  arena_map()->redist_governor()->update(GetSpace().GetSimulationClock(),
                                         collector.cum_collected(),
                                         conv_calculator()->converged());

  /* Collect metrics from/about caches */
  for (auto& c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */
  m_metrics_agg->collect_from_cache_manager(m_cache_manager.get());
  m_cache_manager->reset_metrics();

  /*
   * Manage the static cache. Must be before per-robot processing, so each robot
   * gets their internal object store updated with (potentially)new cache
   * information.
   */
  static_cache_monitor();
  if (m_cache_manager->caches_depleted() > 0) {
    floor()->SetChanged();
  }

  /* Before processing all robots, update the oracles */
  oracle_manager()->update(arena_map());

  /* Process all robots */
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    argos::CFootBotEntity& robot =
        *argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    robot_timestep_process(robot);
  } /* for(&entity_pair..) */

  /* collect metrics from non-robot sources */
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

void depth1_loop_functions::robot_timestep_process(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::base_controller*>(
      &robot.GetControllableEntity().GetController());

  /* get stats from this robot before its state changes */
  auto madaptor =
      robot_metric_extractor_adaptor<depth1_metrics_aggregator>(controller);
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

  /* Watch the robot interact with its environment! */
  auto iadaptor = robot_interactor_adaptor<robot_arena_interactor>(
      controller, GetSpace().GetSimulationClock());
  boost::apply_visitor(iadaptor, m_interactor_map->at(controller->type_index()));
} /* robot_timestep_process() */

argos::CColor depth1_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  rmath::vector2d tmp(plane_pos.GetX(), plane_pos.GetY());
  if (arena_map()->nest().contains_point(tmp)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }
  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks render inside of caches.
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

void depth1_loop_functions::Reset() {
  ndc_push();
  base_loop_functions::Reset();
  m_metrics_agg->reset_all();

  if (auto created = m_cache_manager->create(arena_map()->blocks(),
                                             GetSpace().GetSimulationClock())) {
    arena_map()->caches_add(*created);
    floor()->SetChanged();
  }
  ndc_pop();
} /* Reset() */

__rcsw_pure uint depth1_loop_functions::n_free_blocks(void) const {
  auto accum = [&](uint sum, const auto& b) {
    return sum + (-1 == b->robot_id());
  };

  return std::accumulate(
      arena_map()->blocks().begin(), arena_map()->blocks().end(), 0, accum);
} /* n_free_blocks() */

void depth1_loop_functions::static_cache_monitor(void) {
  if (!arena_map()->caches().empty()) {
    return;
  }
  /*
   * The cache is recreated with a probability that depends on the relative
   * ratio between the # foragers and the # collectors. If there are more
   * foragers than collectors, then the cache will be recreated very quickly. If
   * there are more collectors than foragers, then it will probably not be
   * recreated immediately. And if there are no foragers, there is no chance
   * that the cache could be recreated (trying to emulate depth2 behavior here).
   */
  std::pair<uint, uint> counts{0, 0};
  for (auto& entity_pair : GetSpace().GetEntitiesByType("foot-bot")) {
    auto* robot = argos::any_cast<argos::CFootBotEntity*>(entity_pair.second);
    auto base = dynamic_cast<controller::base_controller*>(
        &robot->GetControllableEntity().GetController());
    auto res =
        boost::apply_visitor(detail::d1_subtask_status_extractor_adaptor(base),
                             m_subtask_status_map->at(base->type_index()));
    counts.first += res.first;
    counts.second += res.second;
  } /* for(&entity..) */

  auto created =
      m_cache_manager->create_conditional(arena_map()->blocks(),
                                          GetSpace().GetSimulationClock(),
                                          counts.first,
                                          counts.second);
  if (created) {
    arena_map()->caches_add(*created);
    __rcsw_unused ds::cell2D& cell = arena_map()->access<arena_grid::kCell>(
        arena_map()->caches()[0]->discrete_loc());
    ER_ASSERT(arena_map()->caches()[0]->n_blocks() == cell.block_count(),
              "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
              arena_map()->caches()[0]->n_blocks(),
              cell.block_count());
    floor()->SetChanged();
  } else {
    ER_INFO(
        "Could not create static cache: n_harvesters=%u,n_collectors=%u,free "
        "blocks=%u",
        counts.first,
        counts.second,
        n_free_blocks());
  }
} /* static_cache_monitor() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wglobal-constructors"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
REGISTER_LOOP_FUNCTIONS(depth1_loop_functions, "depth1_loop_functions");
#pragma clang diagnostic pop
NS_END(depth1, support, fordyca);
