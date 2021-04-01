/**
 * \file d1_loop_functions.cpp
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
#include "fordyca/support/d1/d1_loop_functions.hpp"

#include <boost/mpl/for_each.hpp>

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/controller/operations/applicator.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"
#include "cosm/interactors/applicator.hpp"
#include "cosm/oracle/config/aggregate_oracle_config.hpp"
#include "cosm/pal/argos_convergence_calculator.hpp"
#include "cosm/pal/argos_swarm_iterator.hpp"
#include "cosm/hal/subsystem/config/saa_xml_names.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"
#include "cosm/foraging/block_dist/dispatcher.hpp"

#include "fordyca/controller/cognitive/d1/bitd_dpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_odpo_controller.hpp"
#include "fordyca/controller/cognitive/d1/bitd_omdpo_controller.hpp"
#include "fordyca/events/existing_cache_interactor.hpp"
#include "fordyca/support/d1/d1_metrics_aggregator.hpp"
#include "fordyca/support/d1/robot_arena_interactor.hpp"
#include "fordyca/support/d1/robot_configurer.hpp"
#include "fordyca/support/d1/static_cache_locs_calculator.hpp"
#include "fordyca/support/d1/static_cache_manager.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, d1);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

using configurer_map_type =
    rds::type_map<rmpl::typelist_wrap_apply<controller::d1::typelist,
                                            robot_configurer,
                                            d1_metrics_aggregator>::type>;

template <class Controller>
struct d1_subtask_status_extractor
    : public boost::static_visitor<std::pair<bool, bool>> {
  using controller_type = Controller;

  std::pair<bool, bool> operator()(const Controller* const c) const {
    auto task = dynamic_cast<const cta::polled_task*>(c->current_task());
    return std::make_pair(
        task->name() == tasks::d1::foraging_task::kHarvesterName,
        task->name() == tasks::d1::foraging_task::kCollectorName);
  }
};

/**
 * \struct d1_subtask_status_extractor_adaptor
 * \ingroup support d1
 *
 * \brief Calculate the \ref collector, \ref harvester task counts for d1
 * when a static cache is depleted, for use in determining the static cache
 * respawn probability.
 */
struct d1_subtask_status_extractor_adaptor
    : public boost::static_visitor<std::pair<bool, bool>> {
  explicit d1_subtask_status_extractor_adaptor(
      const controller::foraging_controller* const c)
      : mc_controller(c) {}

  template <typename Controller>
  std::pair<bool, bool>
  operator()(const d1_subtask_status_extractor<Controller>& extractor) const {
    auto cast = dynamic_cast<
        const typename d1_subtask_status_extractor<Controller>::controller_type*>(
        mc_controller);
    return extractor(cast);
  }
  const controller::foraging_controller* const mc_controller;
};

/**
 * \struct functor_maps_initializer
 * \ingroup support d1 detail
 *
 * Convenience class containing initialization for all of the typeid ->
 * boost::variant maps for all controller types that are used throughout
 * initialization and simulation.
 */
struct functor_maps_initializer : public boost::static_visitor<void> {
  RCPPSW_COLD functor_maps_initializer(configurer_map_type* const cmap,
                                       d1_loop_functions* const lf_in)

      : lf(lf_in), config_map(cmap) {}
  template <typename T>
  RCPPSW_COLD void operator()(const T& controller) const {
    typename robot_arena_interactor<T, carena::caching_arena_map>::params p{
      lf->arena_map(),
      lf->m_metrics_agg.get(),
      lf->floor(),
      lf->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>(),
      lf->m_cache_manager.get(),
      lf
    };
    lf->m_interactor_map->emplace(
        typeid(controller),
        robot_arena_interactor<T, carena::caching_arena_map>(p));
    lf->m_metric_extractor_map->emplace(
        typeid(controller),
        ccops::metrics_extract<T, d1_metrics_aggregator>(
            lf->m_metrics_agg.get()));
    lf->m_task_extractor_map->emplace(typeid(controller),
                                      ccops::task_id_extract<T>());
    config_map->emplace(
        typeid(controller),
        robot_configurer<T, d1_metrics_aggregator>(
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
  d1_loop_functions * const lf;
  configurer_map_type* const    config_map;
  /* clang-format on */
};

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
d1_loop_functions::d1_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.d1"),
      m_interactor_map(nullptr),
      m_metric_extractor_map(nullptr),
      m_los_update_map(nullptr),
      m_task_extractor_map(nullptr),
      m_subtask_status_map(nullptr),
      m_metrics_agg(nullptr),
      m_cache_manager(nullptr) {}

d1_loop_functions::~d1_loop_functions(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void d1_loop_functions::init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  config_parse(node);
  auto* aconfig = config()->config_get<caconfig::arena_map_config>();
  if (cfbd::dispatcher::kDistPowerlaw == aconfig->blocks.dist.dist_type) {
    delay_arena_map_init(true);
  }

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* init() */

void d1_loop_functions::shared_init(ticpp::Element& node) {
  d0_loop_functions::shared_init(node);

  /* initialize tasking oracle */
  oracle_init();
} /* shared_init() */

void d1_loop_functions::private_init(void) {
  /* initialize stat collecting */
  auto* output = config()->config_get<cmconfig::output_config>();
  auto* arena = config()->config_get<caconfig::arena_map_config>();

  /* initialize cache handling and create initial cache */
  cache_handling_init(
      config()->config_get<config::caches::caches_config>(),
      &config()->config_get<caconfig::arena_map_config>()->blocks.dist);

  /*
   * Initialize arena map and distribute blocks after creating initial
   * caches. This is only needed for powerlaw distributions.
   */
  auto* vconfig = config()->config_get<cvconfig::visualization_config>();
  if (delay_arena_map_init()) {
    arena_map_init(vconfig);
  }

  m_metrics_agg = std::make_unique<d1_metrics_aggregator>(
      &output->metrics,
      &arena->grid,
      output_root(),
      arena_map()->block_distributor()->block_clustersro().size());
  /* this starts at 0, and ARGoS starts at 1, so sync up */
  m_metrics_agg->timestep_inc_all();

  /*
   * Initialize convergence calculations to include task distribution (if
   * enabled in XML file).
   */
  if (nullptr != conv_calculator()) {
    conv_calculator()->task_dist_entropy_init(std::bind(
        &d1_loop_functions::robot_tasks_extract, this, std::placeholders::_1));
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
   * functors/type maps for all d1 controller types.
   */
  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::d1::typelist>(f_initializer);

  /* configure robots */
  auto cb = [&](auto* controller) {
    ER_ASSERT(config_map.end() != config_map.find(controller->type_index()),
              "Controller '%s' type '%s' not in d1 configuration map",
              controller->GetId().c_str(),
              controller->type_index().name());
    auto applicator = ccops::applicator<controller::foraging_controller,
                                        robot_configurer,
                                        d1_metrics_aggregator>(controller);
    boost::apply_visitor(applicator, config_map.at(controller->type_index()));
  };

  /*
   * Even though this CAN be done in dynamic order, during initialization ARGoS
   * threads are not set up yet so doing dynamicaly causes a deadlock. Also, it
   * only happens once, so it doesn't really matter if it is slow.
   */
  cpal::argos_swarm_iterator::controllers<chal::robot,
                                          controller::foraging_controller,
                                          cpal::iteration_order::ekSTATIC>(
      this, cb, kARGoSRobotType);
} /* private_init() */

void d1_loop_functions::oracle_init(void) {
  auto* oraclep = config()->config_get<coconfig::aggregate_oracle_config>();
  if (nullptr == oraclep || nullptr == oracle()) {
    return;
  }
  if (oraclep->tasking.task_exec_ests || oraclep->tasking.task_interface_ests) {
    /*
     * We just need a copy of the task decomposition graph the robots are
     * using--any robot will do.
     */
    chal::robot& robot0 = *argos::any_cast<chal::robot*>(
        GetSpace().GetEntitiesByType(kARGoSRobotType).begin()->second);
    const auto& controller0 =
        dynamic_cast<controller::cognitive::d1::bitd_dpo_controller&>(
            robot0.GetControllableEntity().GetController());
    auto* bigraph = controller0.executive()->graph();
    oracle()->tasking_oracle(
        std::make_unique<coracle::tasking_oracle>(&oraclep->tasking, bigraph));
  }
} /* oracle_init() */

void d1_loop_functions::cache_handling_init(
    const config::caches::caches_config* cachep,
    const cfconfig::block_dist_config* distp) {
  ER_ASSERT(nullptr != cachep && cachep->static_.enable,
            "FATAL: Caches not enabled in d1 loop functions");
  /*
   * Regardless of how many foragers/etc there are, always create
   * initial caches.
   */
  auto cache_locs = static_cache_locs_calculator()(arena_map(), distp);
  m_cache_manager = std::make_unique<static_cache_manager>(
      cachep, arena_map(), cache_locs, rng());
  cfds::block3D_cluster_vectorro clusters;
  if (!delay_arena_map_init()) {
    clusters = arena_map()->block_distributor()->block_clustersro();
  }
  cache_create_ro_params ccp = { .current_caches = arena_map()->caches(),
                                 .clusters = clusters,
                                 .t = rtypes::timestep(
                                     GetSpace().GetSimulationClock()) };

  cpal::argos_sm_adaptor::led_medium(chsubsystem::config::saa_xml_names::leds_saa);
  bool pre_dist = (nullptr == arena_map()->block_distributor());
  if (auto created =
          m_cache_manager->create(ccp, arena_map()->free_blocks(), pre_dist)) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
  }
} /* cache_handling_init() */

/*******************************************************************************
 * Convergence Calculations Callbacks
 ******************************************************************************/
std::vector<int> d1_loop_functions::robot_tasks_extract(uint) const {
  std::vector<int> v;
  auto cb = [&](auto* controller) {
    auto it = m_task_extractor_map->find(controller->type_index());
    ER_ASSERT(m_task_extractor_map->end() != it,
              "Controller '%s' type '%s' not in d1 task extractor map",
              controller->GetId().c_str(),
              controller->type_index().name());
    auto applicator =
        ccops::applicator<controller::foraging_controller, ccops::task_id_extract>(
            controller);

    v.push_back(boost::apply_visitor(
        applicator, m_task_extractor_map->at(controller->type_index())));
  };
  cpal::argos_swarm_iterator::controllers<chal::robot,
                                          controller::foraging_controller,
                                          cpal::iteration_order::ekSTATIC>(
      this, cb, kARGoSRobotType);
  return v;
} /* robot_tasks_extract() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void d1_loop_functions::pre_step() {
  ndc_push();
  base_loop_functions::pre_step();
  ndc_pop();

  /* Process all robots */
  auto cb = [&](argos::CControllableEntity* robot) {
    ndc_push();
    robot_pre_step(dynamic_cast<chal::robot&>(robot->GetParent()));
    ndc_pop();
  };
  cpal::argos_swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);
} /* pre_step() */

void d1_loop_functions::post_step(void) {
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
              robot_post_step(dynamic_cast<chal::robot&>(robot->GetParent()));
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

  /* update arena map */
  auto* collector =
      m_metrics_agg->get<cfmetrics::block_transportee_metrics_collector>("blocks::"
                                                                         "transpor"
                                                                         "ter");

  /*
   * Update arena map. Free block pickups and nest block drops are covered
   * internally by the arena map in terms of updating block clusters, but task
   * aborts are not, and we need to handle those.
   */
  arena_map()->post_step_update(
      rtypes::timestep(GetSpace().GetSimulationClock()),
      collector->cum_transported(),
      block_op(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);
  block_op(false);

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

void d1_loop_functions::reset(void) {
  ndc_push();
  base_loop_functions::reset();
  m_metrics_agg->reset_all();

  cache_create_ro_params ccp = {
    .current_caches = arena_map()->caches(),
    .clusters = arena_map()->block_distributor()->block_clustersro(),
    .t = rtypes::timestep(GetSpace().GetSimulationClock())
  };

  if (auto created = m_cache_manager->create(ccp, arena_map()->blocks(), false)) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
  }
  ndc_pop();
} /* reset() */

void d1_loop_functions::destroy(void) {
  if (nullptr != m_metrics_agg) {
    m_metrics_agg->finalize_all();
  }
} /* destroy() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void d1_loop_functions::robot_pre_step(chal::robot& robot) {
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
            "Controller '%s' type '%s' not in d1 LOS Update map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto applicator = ccops::applicator<controller::foraging_controller,
                                      ccops::robot_los_update,
                                      rds::grid2D_overlay<cds::cell2D>,
                                      repr::forager_los>(controller);
  boost::apply_visitor(applicator,
                       m_los_update_map->at(controller->type_index()));
} /* robot_pre_step() */

void d1_loop_functions::robot_post_step(chal::robot& robot) {
  auto controller = static_cast<controller::foraging_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Watch the robot interact with its environment after physics have been
   * updated and its controller has run.
   */
  auto it = m_interactor_map->find(controller->type_index());
  ER_ASSERT(m_interactor_map->end() != it,
            "Controller '%s' type '%s' not in d1 interactor map",
            controller->GetId().c_str(),
            controller->type_index().name());
  auto iapplicator = cinteractors::applicator<controller::foraging_controller,
                                              robot_arena_interactor,
                                              carena::caching_arena_map>(
      controller, rtypes::timestep(GetSpace().GetSimulationClock()));

  auto status = boost::apply_visitor(
      iapplicator, m_interactor_map->at(controller->type_index()));

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
   * Mark that a block operation has occured this timestep, and additional
   * processing is needed AFTER all robots have finished their control steps.
   */
  if ((interactor_status::ekFREE_BLOCK_DROP | interactor_status::ekTASK_ABORT) & status) {
    block_op_mtx()->lock();
    block_op(true);
    block_op_mtx()->unlock();
  }

  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto it2 = m_metric_extractor_map->find(controller->type_index());
  ER_ASSERT(m_metric_extractor_map->end() != it2,
            "Controller '%s' type '%s' not in d1 metrics map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto mapplicator = ccops::applicator<controller::foraging_controller,
                                       ccops::metrics_extract,
                                       d1_metrics_aggregator>(controller);
  boost::apply_visitor(mapplicator,
                       m_metric_extractor_map->at(controller->type_index()));
  controller->block_manip_recorder()->reset();
} /* robot_post_step() */

void d1_loop_functions::static_cache_monitor(void) {
  /* nothing to do--all our managed caches exist */
  if (!caches_depleted()) {
    return;
  }

  cache_create_ro_params ccp = {
    .current_caches = arena_map()->caches(),
    .clusters = arena_map()->block_distributor()->block_clustersro(),
    .t = rtypes::timestep(GetSpace().GetSimulationClock())
  };

  if (auto created =
          m_cache_manager->create_conditional(ccp,
                                              arena_map()->free_blocks(),
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

bool d1_loop_functions::caches_depleted(void) const {
  return arena_map()->caches().size() != m_cache_manager->n_managed();
} /* caches_depleted() */

void d1_loop_functions::caches_recreation_task_counts_collect(
    const controller::foraging_controller* const controller) {
  if (caches_depleted()) {
    auto it = m_subtask_status_map->find(controller->type_index());
    ER_ASSERT(m_subtask_status_map->end() != it,
              "Controller '%s' type '%s' not in d1 subtask status map",
              controller->GetId().c_str(),
              controller->type_index().name());

    /*
     * Caches are recreated with a probability that depends on the relative
     * ratio between the # harvesters and the # collectors. If there are more
     * harvesters than collectors, then the cache will be recreated very
     * quickly. If there are more collectors than harvesters, then it will
     * probably not be recreated immediately. And if there are no harvesters,
     * there is no chance that the cache could be recreated (trying to emulate
     * d2 behavior here).
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

REGISTER_LOOP_FUNCTIONS(d1_loop_functions, "d1_loop_functions");

RCPPSW_WARNING_DISABLE_POP()

NS_END(d1, support, fordyca);
