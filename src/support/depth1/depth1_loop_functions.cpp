/**
 * \file depth1_loop_functions.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/controller/operations/applicator.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/metrics/blocks/transport_metrics_collector.hpp"
#include "cosm/interactors/applicator.hpp"
#include "cosm/oracle/config/aggregate_oracle_config.hpp"
#include "cosm/pal/argos_convergence_calculator.hpp"
#include "cosm/pal/argos_swarm_iterator.hpp"
#include "cosm/robots/footbot/config/saa_xml_names.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"

#include "fordyca/controller/depth1/bitd_dpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_odpo_controller.hpp"
#include "fordyca/controller/depth1/bitd_omdpo_controller.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/support/depth1/depth1_metrics_aggregator.hpp"
#include "fordyca/support/depth1/robot_arena_interactor.hpp"
#include "fordyca/support/depth1/robot_configurer.hpp"
#include "fordyca/support/depth1/static_cache_manager.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth1);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

using configurer_map_type =
    rds::type_map<rmpl::typelist_wrap_apply<controller::depth1::typelist,
                                            robot_configurer,
                                            depth1_metrics_aggregator>::type>;

template <class Controller>
struct d1_subtask_status_extractor
    : public boost::static_visitor<std::pair<bool, bool>> {
  using controller_type = Controller;

  std::pair<bool, bool> operator()(const Controller* const c) const {
    auto task = dynamic_cast<const cta::polled_task*>(c->current_task());
    return std::make_pair(
        task->name() == tasks::depth1::foraging_task::kHarvesterName,
        task->name() == tasks::depth1::foraging_task::kCollectorName);
  }
};

/**
 * \struct d1_subtask_status_extractor_adaptor
 * \ingroup support depth1
 *
 * \brief Calculate the \ref collector, \ref harvester task counts for depth1
 * when a static cache is depleted, for use in determining the static cache
 * respawn probability.
 */
struct d1_subtask_status_extractor_adaptor
    : public boost::static_visitor<std::pair<bool, bool>> {
  explicit d1_subtask_status_extractor_adaptor(
      const controller::foraging_controller* const c)
      : mc_controller(c) {}

  template <typename Controller>
  std::pair<bool, bool> operator()(
      const d1_subtask_status_extractor<Controller>& extractor) const {
    auto cast = dynamic_cast<const typename d1_subtask_status_extractor<
        Controller>::controller_type*>(mc_controller);
    return extractor(cast);
  }
  const controller::foraging_controller* const mc_controller;
};

/**
 * \struct functor_maps_initializer
 * \ingroup support depth1 detail
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
    typename robot_arena_interactor<T, carena::caching_arena_map>::params p{
        lf->arena_map(),
        lf->m_metrics_agg.get(),
        lf->floor(),
        lf->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>(),
        lf->m_cache_manager.get(),
        lf};
    lf->m_interactor_map->emplace(
        typeid(controller),
        robot_arena_interactor<T, carena::caching_arena_map>(p));
    lf->m_metric_extractor_map->emplace(
        typeid(controller),
        ccops::metrics_extract<T, depth1_metrics_aggregator>(
            lf->m_metrics_agg.get()));
    lf->m_task_extractor_map->emplace(typeid(controller),
                                      ccops::task_id_extract<T>());
    config_map->emplace(
        typeid(controller),
        robot_configurer<T, depth1_metrics_aggregator>(
            lf->config()->config_get<cvconfig::visualization_config>(),
            lf->oracle(),
            lf->m_metrics_agg.get()));
    lf->m_los_update_map->emplace(
        typeid(controller),
        ccops::robot_los_update<T,
                                rds::grid2D_overlay<cds::cell2D>,
                                repr::forager_los>(
            lf->arena_map()->decoratee().template layer<cds::arena_grid::kCell>()));
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
void depth1_loop_functions::init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* init() */

void depth1_loop_functions::shared_init(ticpp::Element& node) {
  depth0_loop_functions::shared_init(node);

  /* initialize tasking oracle */
  oracle_init();
} /* shared_init() */

void depth1_loop_functions::private_init(void) {
  /* initialize stat collecting */
  auto* output = config()->config_get<cmconfig::output_config>();
  auto* arena = config()->config_get<caconfig::arena_map_config>();


  m_metrics_agg = std::make_unique<depth1_metrics_aggregator>(&output->metrics,
                                                              &arena->grid,
                                                              output_root());
  /* this starts at 0, and ARGoS starts at 1, so sync up */
  m_metrics_agg->timestep_inc_all();

  /* initialize cache handling and create initial cache */
  cache_handling_init(
      config()->config_get<config::caches::caches_config>(),
      &config()->config_get<caconfig::arena_map_config>()->blocks.dist);

  /*
   * Initialize convergence calculations to include task distribution (if
   * enabled in XML file).
   */
  if (nullptr != conv_calculator()) {
    conv_calculator()->task_dist_entropy_init(
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

  /*
   * Intitialize controller interactions with environment via various
   * functors/type maps for all depth1 controller types.
   */
  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::depth1::typelist>(f_initializer);

  /* configure robots */
  auto cb = [&](auto* controller) {
    ER_ASSERT(config_map.end() != config_map.find(controller->type_index()),
              "Controller '%s' type '%s' not in depth1 configuration map",
              controller->GetId().c_str(),
              controller->type_index().name());
    auto applicator = ccops::applicator<controller::foraging_controller,
                                        robot_configurer,
                                        depth1_metrics_aggregator>(controller);
    boost::apply_visitor(applicator, config_map.at(controller->type_index()));
  };

  /*
   * Even though this CAN be done in dynamic order, during initialization ARGoS
   * threads are not set up yet so doing dynamicaly causes a deadlock. Also, it
   * only happens once, so it doesn't really matter if it is slow.
   */
  cpal::argos_swarm_iterator::controllers<argos::CFootBotEntity,
                                          controller::foraging_controller,
                                          cpal::iteration_order::ekSTATIC>(
      this, cb, kARGoSRobotType);
} /* private_init() */

void depth1_loop_functions::oracle_init(void) {
  auto* oraclep = config()->config_get<coconfig::aggregate_oracle_config>();
  if (nullptr == oraclep || nullptr == oracle()) {
    return;
  }
  if (oraclep->tasking.task_exec_ests || oraclep->tasking.task_interface_ests) {
    /*
     * We just need a copy of the task decomposition graph the robots are
     * using--any robot will do.
     */
    argos::CFootBotEntity& robot0 = *argos::any_cast<argos::CFootBotEntity*>(
        GetSpace().GetEntitiesByType(kARGoSRobotType).begin()->second);
    const auto& controller0 =
        dynamic_cast<controller::depth1::bitd_dpo_controller&>(
            robot0.GetControllableEntity().GetController());
    auto* bigraph = controller0.executive()->graph();
    oracle()->tasking_oracle(
        std::make_unique<coracle::tasking_oracle>(&oraclep->tasking, bigraph));
  }
} /* oracle_init() */

void depth1_loop_functions::cache_handling_init(
    const config::caches::caches_config* cachep,
    const cfconfig::block_dist_config* distp) {
  ER_ASSERT(nullptr != cachep && cachep->static_.enable,
            "FATAL: Caches not enabled in depth1 loop functions");
  /*
   * Regardless of how many foragers/etc there are, always create
   * initial caches.
   */
  m_cache_manager = std::make_unique<static_cache_manager>(cachep,
                                                           arena_map(),
                                                           calc_cache_locs(distp),
                                                           rng());

  cache_create_ro_params ccp = {
      .current_caches = arena_map()->caches(),
      .clusters = arena_map()->block_distributor()->block_clusters(),
      .t = rtypes::timestep(GetSpace().GetSimulationClock())};

  cpal::argos_sm_adaptor::led_medium(crfootbot::config::saa_xml_names::leds_saa);
  if (auto created = m_cache_manager->create(ccp, arena_map()->blocks())) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
  }
} /* cache_handling_init() */

std::vector<rmath::vector2d> depth1_loop_functions::calc_cache_locs(
    const cfconfig::block_dist_config* distp) {
  using dispatcher_type = cfbd::dispatcher;

  std::vector<rmath::vector2d> cache_rlocs;
  ER_ASSERT(1 == arena_map()->nests().size(),
            "Multiple nests incompatible with static cache management");
  auto *nest = arena_map()->nest(rtypes::type_uuid(0));

  /*
   * For all block distributions that are supported, each of the static
   * caches is halfway between the center of the nest and a block cluster.
   */
  if (dispatcher_type::kDistSingleSrc == distp->dist_type ||
      dispatcher_type::kDistDualSrc == distp->dist_type) {
    auto clusters = arena_map()->block_distributor()->block_clusters();

    for (auto& c : clusters) {
      cache_rlocs.push_back(
          {(c->xrspan().center() + nest->rcenter2D().x()) / 2.0,
           (c->yrspan().center() + nest->rcenter2D().y()) / 2.0});
    } /* for(i..) */
  } else if (dispatcher_type::kDistQuadSrc == distp->dist_type) {
    /*
     * Quad source is a tricky distribution to use with static caches, so we
     * have to tweak the static cache locations in tandem with the block cluster
     * locations to ensure that no segfaults results from cache/cache or
     * cache/cluster overlap. See FORDYCA#581.
     *
     * Basically we want the cache centers to be halfway between the nest center
     * and each of the block cluster centers (we assume a square arena).
     */
    auto clusters = arena_map()->block_distributor()->block_clusters();
    for (auto& c : clusters) {
      bool on_center_y =
          std::fabs(c->xrspan().center() - nest->rcenter2D().x()) < 0.5;
      bool on_center_x =
          std::fabs(c->yrspan().center() - nest->rcenter2D().y()) < 0.5;
      ER_ASSERT(on_center_y || on_center_x,
                "Cluster@%f,%f not centered in arena X or Y",
                c->xrspan().center(),
                c->yrspan().center());
      if (on_center_x &&
          c->xrspan().center() < nest->rcenter2D().x()) { /* west */
        cache_rlocs.push_back(
            {arena_map()->xrsize() * 0.30, c->yrspan().center()});
      } else if (on_center_x && c->xrspan().center() >
                                    nest->rcenter2D().x()) { /* east */
        cache_rlocs.push_back(
            {arena_map()->xrsize() * 0.675, c->yrspan().center()});
      } else if (on_center_y && c->yrspan().center() <
                                    nest->rcenter2D().y()) { /* south */
        cache_rlocs.push_back(
            {c->xrspan().center(), arena_map()->yrsize() * 0.30});
      } else if (on_center_y && c->yrspan().center() >
                                    nest->rcenter2D().y()) { /* north */
        cache_rlocs.push_back(
            {c->xrspan().center(), arena_map()->yrsize() * 0.675});
      }
    } /* for(i..) */
  } else {
    ER_FATAL_SENTINEL(
        "Block distribution '%s' unsupported for static cache management",
        distp->dist_type.c_str());
  }
  /*
   * For all cache locs, transform real -> discrete to ensure the real and
   * discrete cache centers used during simulation are convertible without loss
   * of precision/weird corner cases. Add 1/2 cell width to them so that the
   * real center is in the center of the host cell, and not the LL corner.
   */
  std::vector<rmath::vector2d> cache_rcenters;
  std::transform(cache_rlocs.begin(),
                 cache_rlocs.end(),
                 std::back_inserter(cache_rcenters),
                 [&](const auto& rloc) {
                   auto tmp = rmath::dvec2zvec(rloc,
                                               arena_map()->grid_resolution().v());
                   rmath::vector2d offset(arena_map()->grid_resolution().v() / 2.0,
                                          arena_map()->grid_resolution().v() / 2.0);
                   auto ll = rmath::zvec2dvec(tmp,
                                              arena_map()->grid_resolution().v());
                   return ll + offset;
                 });
  return cache_rcenters;
} /* calc_cache_locs() */

/*******************************************************************************
 * Convergence Calculations Callbacks
 ******************************************************************************/
std::vector<int> depth1_loop_functions::robot_tasks_extract(uint) const {
  std::vector<int> v;
  auto cb = [&](auto* controller) {
    auto it = m_task_extractor_map->find(controller->type_index());
    ER_ASSERT(m_task_extractor_map->end() != it,
              "Controller '%s' type '%s' not in depth1 task extractor map",
              controller->GetId().c_str(),
              controller->type_index().name());
    auto applicator =
    ccops::applicator<controller::foraging_controller, ccops::task_id_extract>(
            controller);

    v.push_back(boost::apply_visitor(
        applicator, m_task_extractor_map->at(controller->type_index())));
  };
  cpal::argos_swarm_iterator::controllers<argos::CFootBotEntity,
                                          controller::foraging_controller,
                                          cpal::iteration_order::ekSTATIC>(
      this, cb, kARGoSRobotType);
  return v;
} /* robot_tasks_extract() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void depth1_loop_functions::pre_step() {
  ndc_push();
  base_loop_functions::pre_step();
  ndc_pop();

  /* Process all robots */
  auto cb = [&](argos::CControllableEntity* robot) {
    ndc_push();
    robot_pre_step(dynamic_cast<argos::CFootBotEntity&>(robot->GetParent()));
    ndc_pop();
  };
  cpal::argos_swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);
} /* pre_step() */

void depth1_loop_functions::post_step(void) {
  ndc_push();
  base_loop_functions::post_step();
  ndc_pop();

  /*
   * Parallel iteration over the swarm within the following set of ordered
   * tasks:
   *
   * - Metric collection
   * - Task counts collection
   *
   * This has to all be in 1 callback when passing to ARGoS, because we are only
   * allowed 1 usage of ARGoS threads per PreStep()/PostStep() function call.
   */
  auto cb = [&](argos::CControllableEntity* robot) {
    ndc_push();
    robot_post_step(dynamic_cast<argos::CFootBotEntity&>(robot->GetParent()));
    caches_recreation_task_counts_collect(
        &static_cast<controller::foraging_controller&>(robot->GetController()));
    ndc_pop();
  };
  cpal::argos_swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);

  ndc_push();

  /*
   * Manage the static cache and handle cache removal/re-creation as a result of
   * robot interactions with arena.
   */
  static_cache_monitor();
  m_cache_counts.n_harvesters = 0;
  m_cache_counts.n_collectors = 0;

  /* Update block distribution status */
  auto* collector =
      m_metrics_agg->get<cmetrics::blocks::transport_metrics_collector>(
          "blocks::transport");
  arena_map()->redist_governor()->update(
      rtypes::timestep(GetSpace().GetSimulationClock()),
      collector->cum_transported(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);

  /* Collect metrics from/about existing caches */
  for (auto* c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c);
    c->reset_metrics();
  } /* for(&c..) */

  /*
   * Collect metrics from/about zombie caches (caches that have been depleted
   * this timestep). These are not captured by the usual metric collection
   * process as they have been depleted and do not exist anymore in the \ref
   * arena_map::cacheso() array.
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

  m_metrics_agg->metrics_write(rmetrics::output_mode::ekTRUNCATE);
  m_metrics_agg->metrics_write(rmetrics::output_mode::ekCREATE);

  /* Not a clean way to do this in the metrics collectors... */
  if (m_metrics_agg->metrics_write(rmetrics::output_mode::ekAPPEND)) {
    if (nullptr != conv_calculator()) {
      conv_calculator()->reset_metrics();
    }
    tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>()->reset_metrics();
  }
  m_metrics_agg->interval_reset_all();
  m_metrics_agg->timestep_inc_all();

  ndc_pop();
} /* post_step() */

void depth1_loop_functions::reset(void) {
  ndc_push();
  base_loop_functions::reset();
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
} /* reset() */

void depth1_loop_functions::destroy(void) {
  if (nullptr != m_metrics_agg) {
    m_metrics_agg->finalize_all();
  }
} /* destroy() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void depth1_loop_functions::robot_pre_step(argos::CFootBotEntity& robot) {
  auto controller = static_cast<controller::foraging_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Update robot position, time. This can't be done as part of the robot's
   * control step because we need access to information only available in the
   * loop functions.
   */
  controller->sensing_update(rtypes::timestep(GetSpace().GetSimulationClock()),
                             arena_map()->grid_resolution());

  /* Send robot its new LOS */
  auto it = m_los_update_map->find(controller->type_index());
  ER_ASSERT(m_los_update_map->end() != it,
            "Controller '%s' type '%s' not in depth1 LOS Update map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto applicator = ccops::applicator<controller::foraging_controller,
                                      ccops::robot_los_update,
                                      rds::grid2D_overlay<cds::cell2D>,
                                      repr::forager_los>(controller);
  boost::apply_visitor(applicator,
                       m_los_update_map->at(controller->type_index()));
} /* robot_pre_step() */

void depth1_loop_functions::robot_post_step(argos::CFootBotEntity& robot) {
  auto controller = static_cast<controller::foraging_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Watch the robot interact with its environment after physics have been
   * updated and its controller has run.
   */
  auto it = m_interactor_map->find(controller->type_index());
  ER_ASSERT(m_interactor_map->end() != it,
            "Controller '%s' type '%s' not in depth1 interactor map",
            controller->GetId().c_str(),
            controller->type_index().name());
  auto iapplicator =
      cinteractors::applicator<controller::foraging_controller,
                               robot_arena_interactor,
                               carena::caching_arena_map>(
          controller, rtypes::timestep(GetSpace().GetSimulationClock()));

  auto status =
      boost::apply_visitor(iapplicator,
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
   * See FORDYCA#577.
   */
  if (interactor_status::ekNO_EVENT != status && nullptr != oracle()) {
    oracle()->update(arena_map());
  }

  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto it2 = m_metric_extractor_map->find(controller->type_index());
  ER_ASSERT(m_metric_extractor_map->end() != it2,
            "Controller '%s' type '%s' not in depth1 metrics map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto mapplicator = ccops::applicator<controller::foraging_controller,
                                       ccops::metrics_extract,
                                       depth1_metrics_aggregator>(controller);
  boost::apply_visitor(mapplicator,
                       m_metric_extractor_map->at(controller->type_index()));
  controller->block_manip_recorder()->reset();
} /* robot_post_step() */

void depth1_loop_functions::static_cache_monitor(void) {
  /* nothing to do--all our managed caches exist */
  if (!caches_depleted()) {
    return;
  }

  cache_create_ro_params ccp = {
      .current_caches = arena_map()->caches(),
      .clusters = arena_map()->block_distributor()->block_clusters(),
      .t = rtypes::timestep(GetSpace().GetSimulationClock())};

  if (auto created =
          m_cache_manager->create_conditional(ccp,
                                              arena_map()->blocks(),
                                              m_cache_counts.n_harvesters,
                                              m_cache_counts.n_collectors)) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
    return;
  }
  ER_INFO("Could not create static caches: n_harvesters=%u,n_collectors=%u",
          m_cache_counts.n_harvesters.load(),
          m_cache_counts.n_collectors.load());
} /* static_cache_monitor() */

bool depth1_loop_functions::caches_depleted(void) const {
  return arena_map()->caches().size() != m_cache_manager->n_managed();
} /* caches_depleted() */

void depth1_loop_functions::caches_recreation_task_counts_collect(
    const controller::foraging_controller* const controller) {
  if (caches_depleted()) {
    auto it = m_subtask_status_map->find(controller->type_index());
    ER_ASSERT(m_subtask_status_map->end() != it,
              "Controller '%s' type '%s' not in depth1 subtask status map",
              controller->GetId().c_str(),
              controller->type_index().name());

    /*
     * Caches are recreated with a probability that depends on the relative
     * ratio between the # harvesters and the # collectors. If there are more
     * harvesters than collectors, then the cache will be recreated very
     * quickly. If there are more collectors than harvesters, then it will
     * probably not be recreated immediately. And if there are no harvesters,
     * there is no chance that the cache could be recreated (trying to emulate
     * depth2 behavior here).
     */
    auto [is_harvester, is_collector] = boost::apply_visitor(
        detail::d1_subtask_status_extractor_adaptor(controller),
        m_subtask_status_map->at(controller->type_index()));
    m_cache_counts.n_harvesters += is_harvester;
    m_cache_counts.n_collectors += is_collector;
  }
} /* caches_recreation_task_counts_collect() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_LOOP_FUNCTIONS(depth1_loop_functions, "depth1_loop_functions");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth1, support, fordyca);
