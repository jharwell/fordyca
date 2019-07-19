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

#include "fordyca/config/arena/arena_map_config.hpp"
#include "fordyca/config/output_config.hpp"
#include "fordyca/config/visualization_config.hpp"
#include "fordyca/controller/depth2/grp_dpo_controller.hpp"
#include "fordyca/controller/depth2/grp_mdpo_controller.hpp"
#include "fordyca/controller/depth2/grp_odpo_controller.hpp"
#include "fordyca/controller/depth2/grp_omdpo_controller.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
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
#include "fordyca/support/swarm_iterator.hpp"

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
  RCSW_COLD functor_maps_initializer(configurer_map_type* const cmap,
                                     depth2_loop_functions* const lf_in)

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
        robot_metric_extractor<depth2_metrics_aggregator, T>(
            lf->m_metrics_agg.get()));
    if (nullptr != lf->oracle_manager()) {
      config_map->emplace(
          typeid(controller),
          robot_configurer<T, depth2_metrics_aggregator>(
              lf->config()->config_get<config::visualization_config>(),
              lf->oracle_manager()->entities_oracle(),
              lf->oracle_manager()->tasking_oracle(),
              lf->m_metrics_agg.get()));
    } else {
      config_map->emplace(
          typeid(controller),
          robot_configurer<T, depth2_metrics_aggregator>(
              lf->config()->config_get<config::visualization_config>(),
              nullptr,
              nullptr,
              lf->m_metrics_agg.get()));
    }

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
} /* shared_init() */

void depth2_loop_functions::private_init(void) {
  /* initialize stat collecting */
  auto* arenap = config()->config_get<config::arena::arena_map_config>();

  config::output_config output =
      *config()->config_get<const config::output_config>();
  output.metrics.arena_grid = arenap->grid;
  m_metrics_agg = std::make_unique<depth2_metrics_aggregator>(&output.metrics,
                                                              output_root());

  /* initialize cache handling */
  auto* cachep = config()->config_get<config::caches::caches_config>();
  cache_handling_init(cachep);

  /*
   * Initialize convergence calculations to include task distribution (not
   * included by default).
   */
  if (nullptr != conv_calculator()) {
    conv_calculator()->task_dist_init(
        std::bind(&depth2_loop_functions::robot_tasks_extract,
                  this,
                  std::placeholders::_1));
  }

  /*
   * Intitialize robot interactions with environment wth various functors/type
   * maps.
   */
  m_interactor_map = std::make_unique<interactor_map_type>();
  m_metric_extractor_map = std::make_unique<metric_extractor_map_type>();
  m_los_update_map = std::make_unique<los_updater_map_type>();
  m_task_extractor_map = std::make_unique<task_extractor_map_type>();

  /* only needed for initialization, so not a member */
  auto config_map = detail::configurer_map_type();

  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::depth2::typelist>(f_initializer);

  /* configure robots */
  swarm_iterator::controllers(this, [&](auto* controller) {
    boost::apply_visitor(detail::robot_configurer_adaptor(controller),
                         config_map.at(controller->type_index()));
  });
} /* private_init() */

void depth2_loop_functions::cache_handling_init(
    const config::caches::caches_config* const cachep) {
  ER_ASSERT(nullptr != cachep && cachep->dynamic.enable,
            "FATAL: Caches not enabled in depth2 loop functions");
  m_cache_manager =
      std::make_unique<dynamic_cache_manager>(cachep, &arena_map()->decoratee());

  cache_creation_handle(false);
} /* cache_handlng_init() */

/*******************************************************************************
 * Convergence Calculations Callbacks
 ******************************************************************************/
std::vector<int> depth2_loop_functions::robot_tasks_extract(uint) const {
  std::vector<int> v;
  swarm_iterator::controllers(this, [&](auto* controller) {
    v.push_back(boost::apply_visitor(robot_task_extractor_adaptor(controller),
                                     m_task_extractor_map->at(
                                         controller->type_index())));
  });
  return v;
} /* robot_tasks_extract() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void depth2_loop_functions::PreStep() {
  ndc_push();

  base_loop_functions::PreStep();

  auto& collector = static_cast<metrics::blocks::transport_metrics_collector&>(
      *(*m_metrics_agg)["blocks::transport"]);
  arena_map()->redist_governor()->update(
      rtypes::timestep(GetSpace().GetSimulationClock()),
      collector.cum_collected(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);

  /* Collect metrics from/about caches */
  for (auto& c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c.get());
    c->reset_metrics();
  } /* for(&c..) */

  m_metrics_agg->collect_from_cache_manager(m_cache_manager.get());
  m_cache_manager->reset_metrics();

  /* Process all robots */
  swarm_iterator::robots(this,
                         [&](auto* robot) { robot_timestep_process(*robot); });

  /* handle cache removal as a result of robot interactions with arena */
  if (m_cache_manager->caches_depleted() > 0) {
    floor()->SetChanged();
  }
  /* collect metrics from non-robot sources */
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
  utils::set_robot_pos<decltype(*controller)>(robot,
                                              arena_map()->grid_resolution());
  utils::set_robot_tick<decltype(*controller)>(
      robot, rtypes::timestep(GetSpace().GetSimulationClock()));
  boost::apply_visitor(robot_los_updater_adaptor(controller),
                       m_los_update_map->at(controller->type_index()));

  /*
   * Watch the robot interact with its environment!
   *
   * If said interaction results in a block being dropped in a new cache, then
   * we need to re-run dynamic cache creation.
   */
  auto iadaptor =
      robot_interactor_adaptor<robot_arena_interactor, interactor_status>(
          controller, rtypes::timestep(GetSpace().GetSimulationClock()));
  auto status =
      boost::apply_visitor(iadaptor,
                           m_interactor_map->at(controller->type_index()));
  if (interactor_status::ekNoEvent != status) {
    if (interactor_status::ekNewCacheBlockDrop & status) {
      bool ret = cache_creation_handle(true);
      if (!ret) {
        ER_WARN("Unable to create cache after block drop in new cache");
      }
    }

    /*
     * The oracle does not have up-to-date information about all caches in the
     * arena now that one has been created, so we need to update the oracle in
     * the middle of processing robots. This is not an issue in depth1, because
     * caches are always created AFTER processing all robots for a timestep.
     *
     * It also does not necessarily have up-to-date information about all blocks
     * in the arena, as a robot could have dropped a block when it aborted its
     * current task.
     */
    if (nullptr != oracle_manager()) {
      oracle_manager()->update(arena_map());
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

void depth2_loop_functions::Destroy(void) {
  if (nullptr != m_metrics_agg) {
    m_metrics_agg->finalize_all();
  }
} /* Destroy() */

bool depth2_loop_functions::cache_creation_handle(bool on_drop) {
  auto* cachep = config()->config_get<config::caches::caches_config>();
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
                              rtypes::timestep(GetSpace().GetSimulationClock()));
  if (created) {
    arena_map()->caches_add(*created, this);
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
