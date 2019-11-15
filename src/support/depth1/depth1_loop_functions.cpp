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

#include "rcppsw/ta/bi_tdgraph_executive.hpp"
#include "rcppsw/ta/ds/bi_tdgraph.hpp"

#include "fordyca/config/arena/arena_map_config.hpp"
#include "fordyca/config/oracle/oracle_manager_config.hpp"
#include "fordyca/config/output_config.hpp"
#include "fordyca/config/visualization_config.hpp"
#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/repr/block_cluster.hpp"
#include "fordyca/support/block_dist/base_distributor.hpp"
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
#include "fordyca/support/swarm_iterator.hpp"

#include "cosm/convergence/convergence_calculator.hpp"

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
    : public boost::static_visitor<std::pair<bool, bool>> {
  using controller_type = ControllerType;

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
  RCSW_COLD functor_maps_initializer(configurer_map_type* const cmap,
                                     depth1_loop_functions* const lf_in)

      : lf(lf_in), config_map(cmap) {}
  template <typename T>
  RCSW_COLD void operator()(const T& controller) const {
    typename robot_arena_interactor<T>::params p{lf->arena_map(),
                                                 lf->m_metrics_agg.get(),
                                                 lf->floor(),
                                                 lf->tv_manager(),
                                                 lf->m_cache_manager.get(),
                                                 lf};
    lf->m_interactor_map->emplace(typeid(controller),
                                  robot_arena_interactor<T>(p));
    lf->m_metric_extractor_map->emplace(
        typeid(controller),
        robot_metric_extractor<depth1_metrics_aggregator, T>(
            lf->m_metrics_agg.get()));
    if (nullptr != lf->oracle_manager()) {
      config_map->emplace(
          typeid(controller),
          robot_configurer<T, depth1_metrics_aggregator>(
              lf->config()->config_get<config::visualization_config>(),
              lf->oracle_manager()->entities_oracle(),
              lf->oracle_manager()->tasking_oracle(),
              lf->m_metrics_agg.get()));
    } else {
      config_map->emplace(
          typeid(controller),
          robot_configurer<T, depth1_metrics_aggregator>(
              lf->config()->config_get<config::visualization_config>(),
              nullptr,
              nullptr,
              lf->m_metrics_agg.get()));
    }
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

  /* initialize tasking oracle */
  oracle_init();
} /* shared_init() */

void depth1_loop_functions::private_init(void) {
  /* initialize stat collecting */
  auto* arenap = config()->config_get<config::arena::arena_map_config>();
  config::output_config output =
      *config()->config_get<const config::output_config>();
  output.metrics.arena_grid = arenap->grid;
  m_metrics_agg = std::make_unique<depth1_metrics_aggregator>(&output.metrics,
                                                              output_root());

  /* initialize cache handling and create initial cache */
  cache_handling_init(
      config()->config_get<config::caches::caches_config>(),
      &config()->config_get<config::arena::arena_map_config>()->blocks.dist);

  /*
   * Initialize convergence calculations to include task distribution (not
   * included by default).
   */
  if (nullptr != conv_calculator()) {
    conv_calculator()->task_dist_init(
        std::bind(&depth1_loop_functions::robot_tasks_extract,
                  this,
                  std::placeholders::_1));
  }

  /*
   * Intitialize robot interactions with environment via various functors/type
   * maps.
   */
  m_interactor_map = std::make_unique<interactor_map_type>();
  m_metric_extractor_map = std::make_unique<metric_extractor_map_type>();
  m_los_update_map = std::make_unique<los_updater_map_type>();
  m_task_extractor_map = std::make_unique<task_extractor_map_type>();
  m_subtask_status_map = std::make_unique<detail::d1_subtask_status_map_type>();

  /* only needed for initialization, so not a member */
  auto config_map = detail::configurer_map_type();

  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::depth1::typelist>(f_initializer);

  /* configure robots */
  auto cb = [&](auto* controller) {
    boost::apply_visitor(detail::robot_configurer_adaptor(controller),
                         config_map.at(controller->type_index()));
  };
  swarm_iterator::controllers<argos::CFootBotEntity,
                              swarm_iterator::dynamic_order>(this,
                                                             cb,
                                                             "foot-bot");
} /* private_init() */

void depth1_loop_functions::oracle_init(void) {
  auto* oraclep = config()->config_get<config::oracle::oracle_manager_config>();
  if (nullptr == oraclep || nullptr == oracle_manager()) {
    return;
  }
  if (oraclep->tasking.task_exec_ests || oraclep->tasking.task_interface_ests) {
    /*
     * We just need a copy of the task decomposition graph the robots are
     * using--any robot will do.
     */
    argos::CFootBotEntity& robot0 = *argos::any_cast<argos::CFootBotEntity*>(
        GetSpace().GetEntitiesByType("foot-bot").begin()->second);
    const auto& controller0 =
        dynamic_cast<controller::depth1::bitd_dpo_controller&>(
            robot0.GetControllableEntity().GetController());
    auto* bigraph = dynamic_cast<const rta::ds::bi_tdgraph*>(
        controller0.executive()->graph());
    oracle_manager()->tasking_oracle(
        std::make_unique<support::oracle::tasking_oracle>(&oraclep->tasking,
                                                          bigraph));
  }
} /* oracle_init() */

void depth1_loop_functions::cache_handling_init(
    const config::caches::caches_config* cachep,
    const config::arena::block_dist_config* distp) {
  ER_ASSERT(nullptr != cachep && cachep->static_.enable,
            "FATAL: Caches not enabled in depth1 loop functions");
  /*
   * Regardless of how many foragers/etc there are, always create
   * initial caches.
   */
  m_cache_manager = std::make_unique<static_cache_manager>(
      cachep, &arena_map()->decoratee(), calc_cache_locs(distp), rng());

  cache_create_ro_params ccp = {
      .current_caches = arena_map()->caches(),
      .clusters = arena_map()->block_distributor()->block_clusters(),
      .t = rtypes::timestep(GetSpace().GetSimulationClock())};
  if (auto created = m_cache_manager->create(ccp, arena_map()->blocks())) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
  }
} /* cache_handling_init() */

std::vector<rmath::vector2d> depth1_loop_functions::calc_cache_locs(
    const config::arena::block_dist_config* distp) {
  std::vector<rmath::vector2d> cache_locs;

  /*
   * For all block distributions that are supported, and each of the static
   * caches is halfway between the center of the nest and a block cluster.
   */
  if (support::block_dist::dispatcher::kDistSingleSrc == distp->dist_type ||
      support::block_dist::dispatcher::kDistDualSrc == distp->dist_type) {
    auto clusters = arena_map()->block_distributor()->block_clusters();
    for (auto& c : clusters) {
      cache_locs.push_back(
          {(c->xspan().center() + arena_map()->nest().rloc().x()) / 2.0,
           (c->yspan().center() + arena_map()->nest().rloc().y()) / 2.0});
    } /* for(i..) */
  } else if (support::block_dist::dispatcher::kDistQuadSrc == distp->dist_type) {
    /*
     * Quad source is a tricky distribution to use with static caches, so we
     * have to tweak the static cache locations in tandem with the block cluster
     * locations to ensure that no segfaults results from cache/cache or
     * cache/cluster overlap. See #581.
     *
     * Basically we want the cache centers to be halfway between the nest center
     * and each of the block cluster centers (we assume a square arena).
     */
    auto clusters = arena_map()->block_distributor()->block_clusters();
    for (auto& c : clusters) {
      bool on_center_y =
          std::fabs(c->xspan().center() - arena_map()->nest().rloc().x()) < 0.5;
      bool on_center_x =
          std::fabs(c->yspan().center() - arena_map()->nest().rloc().y()) < 0.5;
      ER_ASSERT(on_center_y || on_center_x,
                "Cluster@%f,%f not centered in arena X or Y",
                c->xspan().center(),
                c->yspan().center());
      if (on_center_x &&
          c->xspan().center() < arena_map()->nest().rloc().x()) { /* west */
        cache_locs.push_back(
            {arena_map()->xrsize() * 0.30, c->yspan().center()});
      } else if (on_center_x && c->xspan().center() >
                                    arena_map()->nest().rloc().x()) { /* east */
        cache_locs.push_back(
            {arena_map()->xrsize() * 0.675, c->yspan().center()});
      } else if (on_center_y && c->yspan().center() <
                                    arena_map()->nest().rloc().y()) { /* south */
        cache_locs.push_back(
            {c->xspan().center(), arena_map()->yrsize() * 0.30});
      } else if (on_center_y && c->yspan().center() >
                                    arena_map()->nest().rloc().y()) { /* north */
        cache_locs.push_back(
            {c->xspan().center(), arena_map()->yrsize() * 0.675});
      }
    } /* for(i..) */
  } else {
    ER_FATAL_SENTINEL(
        "Block distribution '%s' unsupported for static cache management",
        distp->dist_type.c_str());
  }
  return cache_locs;
} /* calc_cache_locs() */

/*******************************************************************************
 * Convergence Calculations Callbacks
 ******************************************************************************/
std::vector<int> depth1_loop_functions::robot_tasks_extract(uint) const {
  std::vector<int> v;
  auto cb = [&](const auto* controller) {
    v.push_back(boost::apply_visitor(
        robot_task_extractor_adaptor(controller),
        m_task_extractor_map->at(controller->type_index())));
  };
  swarm_iterator::controllers<argos::CFootBotEntity,
                              swarm_iterator::static_order>(this,
                                                            cb,
                                                            "foot-bot");
  return v;
} /* robot_tasks_extract() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void depth1_loop_functions::PreStep() {
  ndc_push();
  base_loop_functions::PreStep();

  /* Process all robots */
  auto cb = [&](auto* robot) { robot_pre_step(*robot); };
  swarm_iterator::robots<argos::CFootBotEntity,
                         swarm_iterator::dynamic_order>(this,
                                                        cb,
                                                        "foot-bot");
  ndc_pop();
} /* PreStep() */

void depth1_loop_functions::PostStep(void) {
  ndc_push();
  base_loop_functions::PostStep();

  /* Process all robots: interact with environment then collect metrics */
  auto cb = [&](auto* robot) { robot_post_step(*robot); };
  swarm_iterator::robots<argos::CFootBotEntity,
                         swarm_iterator::dynamic_order>(this,
                                                        cb,
                                                        "foot-bot");

  /*
   * Manage the static cache and handle cache removal as a result of robot
   * interactions with arena.
   */
  static_cache_monitor();
  if (m_cache_manager->caches_depleted() > 0) {
    floor()->SetChanged();
  }

  /* Update block distribution status */
  auto& collector = static_cast<metrics::blocks::transport_metrics_collector&>(
      *(*m_metrics_agg)["blocks::transport"]);
  arena_map()->redist_governor()->update(
      rtypes::timestep(GetSpace().GetSimulationClock()),
      collector.cum_collected(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);

  /* Collect metrics from/about existing caches */
  for (auto& c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */

  /*
   * Collect metrics from/about zombie caches (caches that have been depleted
   * this timestep). These are not captured by the usual metric collection
   * process as they have been depleted and do not exist anymore in the \ref
   * arena_map::caches() array.
   */
  for (auto& c : arena_map()->zombie_caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */
  arena_map()->zombie_caches_clear();

  m_metrics_agg->collect_from_cache_manager(m_cache_manager.get());
  m_cache_manager->reset_metrics();

  /* Collect metrics from loop functions */
  m_metrics_agg->collect_from_loop(this);

  /* Not a clean way to do this in the convergence metrics collector... */
  if (m_metrics_agg->metrics_write_all(
          rtypes::timestep(GetSpace().GetSimulationClock())) &&
      nullptr != conv_calculator()) {
    conv_calculator()->reset_metrics();
  }
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->interval_reset_all();

  ndc_pop();
} /* PostStep() */

void depth1_loop_functions::Reset() {
  ndc_push();
  base_loop_functions::Reset();
  m_metrics_agg->reset_all();

  cache_create_ro_params ccp = {
      .current_caches = arena_map()->caches(),
      .clusters = arena_map()->block_distributor()->block_clusters(),
      .t = rtypes::timestep(GetSpace().GetSimulationClock())};

  if (auto created = m_cache_manager->create(ccp, arena_map()->blocks())) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
  }
  ndc_pop();
} /* Reset() */

void depth1_loop_functions::Destroy(void) {
  if (nullptr != m_metrics_agg) {
    m_metrics_agg->finalize_all();
  }
} /* Destroy() */

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

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void depth1_loop_functions::robot_pre_step(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::base_controller*>(
      &robot.GetControllableEntity().GetController());

  /* Set robot position, time, and send it its new LOS */
  utils::set_robot_pos<decltype(*controller)>(robot,
                                              arena_map()->grid_resolution());
  utils::set_robot_tick<decltype(*controller)>(
      robot, rtypes::timestep(GetSpace().GetSimulationClock()));
  boost::apply_visitor(robot_los_updater_adaptor(controller),
                       m_los_update_map->at(controller->type_index()));
} /* robot_pre_step() */

void depth1_loop_functions::robot_post_step(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::base_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Watch the robot interact with its environment after physics have been
   * updated and its controller has run.
   */
  auto iadaptor =
      robot_interactor_adaptor<robot_arena_interactor, interactor_status>(
          controller, rtypes::timestep(GetSpace().GetSimulationClock()));
  auto status =
      boost::apply_visitor(iadaptor,
                           m_interactor_map->at(controller->type_index()));
  /*
   * The oracle does not necessarily have up-to-date information about all
   * blocks in the arena, as a robot could have dropped a block in the nest or
   * picked one up, so its version of the set of free blocks in the arena is out
   * of date. Robots processed *after* the robot that caused the event need
   * the correct free block set to be available from the oracle upon request, to
   * avoid asserts during on debug builds. On optimized builds the asserts are
   * ignored/compiled out, which is not a problem, because they LOS processing
   * errors that can result are transient and are corrected the next
   * timestep.
   *
   * This is not a problem for caches, because caches are created (if needed)
   * after *all* robots have been processed for the given timestep.
   *
   * See #577.
   */
  if (interactor_status::ekNoEvent != status && nullptr != oracle_manager()) {
    oracle_manager()->update(arena_map());
  }

  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto madaptor =
      robot_metric_extractor_adaptor<depth1_metrics_aggregator>(controller);
  boost::apply_visitor(madaptor,
                       m_metric_extractor_map->at(controller->type_index()));
  controller->block_manip_collator()->reset();
} /* robot_post_step() */

void depth1_loop_functions::static_cache_monitor(void) {
  /* nothing to do--all our managed caches exist */
  if (arena_map()->caches().size() == m_cache_manager->n_managed()) {
    return;
  }
  /*
   * Caches are recreated with a probability that depends on the relative ratio
   * between the # harvesters and the # collectors. If there are more harvesters
   * than collectors, then the cache will be recreated very quickly. If there
   * are more collectors than harvesters, then it will probably not be recreated
   * immediately. And if there are no harvesters, there is no chance that the
   * cache could be recreated (trying to emulate depth2 behavior here).
   */
  std::pair<uint, uint> counts{0, 0};
  auto cb = [&](const auto* controller) {
    auto [is_harvester, is_collector] = boost::apply_visitor(
        detail::d1_subtask_status_extractor_adaptor(controller),
        m_subtask_status_map->at(controller->type_index()));
    counts.first += is_harvester;
    counts.second += is_collector;
  };
  swarm_iterator::controllers<argos::CFootBotEntity,
                              swarm_iterator::static_order>(this,
                                                            cb,
                                                            "foot-bot");
  cache_create_ro_params ccp = {
      .current_caches = arena_map()->caches(),
      .clusters = arena_map()->block_distributor()->block_clusters(),
      .t = rtypes::timestep(GetSpace().GetSimulationClock())};

  if (auto created = m_cache_manager->create_conditional(
          ccp, arena_map()->blocks(), counts.first, counts.second)) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
    return;
  }
  ER_INFO(
      "Could not create static caches: n_harvesters=%u,n_collectors=%u,free "
      "blocks=%zu",
      counts.first,
      counts.second,
      utils::free_blocks_calc(arena_map()->caches(), arena_map()->blocks())
          .size());
} /* static_cache_monitor() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_LOOP_FUNCTIONS(depth1_loop_functions, "depth1_loop_functions");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth1, support, fordyca);
