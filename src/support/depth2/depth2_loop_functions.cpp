/**
 * \file depth2_loop_functions.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/controller/operations/applicator.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"
#include "cosm/metrics/blocks/transport_metrics_collector.hpp"
#include "cosm/interactors/applicator.hpp"
#include "cosm/pal/argos_convergence_calculator.hpp"
#include "cosm/pal/argos_swarm_iterator.hpp"
#include "cosm/robots/footbot/config/saa_xml_names.hpp"
#include "cosm/ta/bi_tdgraph_executive.hpp"
#include "cosm/ta/ds/bi_tdgraph.hpp"

#include "fordyca/controller/depth2/birtd_dpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_mdpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_odpo_controller.hpp"
#include "fordyca/controller/depth2/birtd_omdpo_controller.hpp"
#include "fordyca/support/depth2/depth2_metrics_aggregator.hpp"
#include "fordyca/support/depth2/dynamic_cache_manager.hpp"
#include "fordyca/support/depth2/robot_arena_interactor.hpp"
#include "fordyca/support/depth2/robot_configurer.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

using configurer_map_type =
    rds::type_map<rmpl::typelist_wrap_apply<controller::depth2::typelist,
                                            robot_configurer,
                                            depth2_metrics_aggregator>::type>;

/**
 * \struct functor_maps_initializer
 * \ingroup support depth2 detail
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
        ccops::metrics_extract<T, depth2_metrics_aggregator>(
            lf->m_metrics_agg.get()));
    config_map->emplace(
        typeid(controller),
        robot_configurer<T, depth2_metrics_aggregator>(
            lf->config()->config_get<cvconfig::visualization_config>(),
            lf->oracle(),
            lf->m_metrics_agg.get()));
    lf->m_los_update_map->emplace(
        typeid(controller),
        ccops::robot_los_update<T,
                                rds::grid2D_overlay<cds::cell2D>,
                                repr::forager_los>(
            lf->arena_map()->decoratee().template layer<cds::arena_grid::kCell>()));
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
void depth2_loop_functions::init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* init() */

void depth2_loop_functions::shared_init(ticpp::Element& node) {
  depth1_loop_functions::shared_init(node);
} /* shared_init() */

void depth2_loop_functions::private_init(void) {
  /* initialize stat collecting */
  auto* output = config()->config_get<cmconfig::output_config>();
  auto* arena = config()->config_get<caconfig::arena_map_config>();

  m_metrics_agg = std::make_unique<depth2_metrics_aggregator>(&output->metrics,
                                                              &arena->grid,
                                                              output_root());
  /* this starts at 0, and ARGoS starts at 1, so sync up */
  m_metrics_agg->timestep_inc_all();

  /* initialize cache handling */
  auto* cachep = config()->config_get<config::caches::caches_config>();
  cache_handling_init(cachep);

  /*
   * Initialize convergence calculations to include task distribution (not
   * included by default).
   */
  if (nullptr != conv_calculator()) {
    conv_calculator()->task_dist_entropy_init(
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
  auto cb = [&](auto* controller) {
    ER_ASSERT(config_map.end() != config_map.find(controller->type_index()),
              "Controller '%s' type '%s' not in depth2 configuration map",
              controller->GetId().c_str(),
              controller->type_index().name());

    auto applicator = ccops::applicator<controller::foraging_controller,
                                        robot_configurer,
                                        depth2_metrics_aggregator>(controller);
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

void depth2_loop_functions::cache_handling_init(
    const config::caches::caches_config* const cachep) {
  ER_ASSERT(nullptr != cachep && cachep->dynamic.enable,
            "FATAL: Caches not enabled in depth2 loop functions");
  m_cache_manager = std::make_unique<dynamic_cache_manager>(
      cachep, &arena_map()->decoratee(), rng());
  argos_sm_adaptor::led_medium(crfootbot::config::saa_xml_names().leds_saa);
  cache_creation_handle(false);
} /* cache_handlng_init() */

/*******************************************************************************
 * Convergence Calculations Callbacks
 ******************************************************************************/
std::vector<int> depth2_loop_functions::robot_tasks_extract(uint) const {
  std::vector<int> v;
  auto cb = [&](auto* controller) {
    auto it = m_task_extractor_map->find(controller->type_index());
    ER_ASSERT(m_task_extractor_map->end() != it,
              "Controller '%s' type '%s' not in depth2 task extraction map",
              controller->GetId().c_str(),
              controller->type_index().name());
    auto applicator =
        ccops::applicator<controller::foraging_controller, robot_task_extractor>(
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
void depth2_loop_functions::pre_step() {
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

void depth2_loop_functions::post_step(void) {
  ndc_push();
  base_loop_functions::post_step();
  ndc_pop();

  /* Process all robots: environment interactions then collect metrics */
  auto cb = [&](argos::CControllableEntity* robot) {
    ndc_push();
    robot_post_step(dynamic_cast<argos::CFootBotEntity&>(robot->GetParent()));
    ndc_pop();
  };
  cpal::argos_swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);

  ndc_push();
  /*
   * Run dynamic cache creation if it was triggered. We don't wan't to run it
   * unconditionally each timestep, because it is VERRRYYYYY expensive to
   * compute.
   */
  if (m_dynamic_cache_create) {
    if (!cache_creation_handle(true)) {
      ER_WARN("Unable to create cache after block drop(s) in new cache");
    }
    m_dynamic_cache_create = false;
  }

  /* Update block distribution status */
  auto* collector =
      m_metrics_agg->get<cmetrics::blocks::transport_metrics_collector>(
          "blocks::transport");
  arena_map()->redist_governor()->update(
      rtypes::timestep(GetSpace().GetSimulationClock()),
      collector->cum_transported(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);

  /* Collect metrics from/about caches */
  for (auto* c : arena_map()->caches()) {
    m_metrics_agg->collect_from_cache(c);
    c->reset_metrics();
  } /* for(&c..) */

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

void depth2_loop_functions::reset(void) {
  ndc_push();
  base_loop_functions::reset();
  m_metrics_agg->reset_all();
  cache_creation_handle(false);
  ndc_pop();
}

void depth2_loop_functions::destroy(void) {
  if (nullptr != m_metrics_agg) {
    m_metrics_agg->finalize_all();
  }
} /* destroy() */

argos::CColor depth2_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  rmath::vector2d tmp(plane_pos.GetX(), plane_pos.GetY());

  /* check if the point is inside any of the nests */
  for (auto *nest : arena_map()->nests()) {
    if (nest->contains_point(tmp)) {
      return argos::CColor(nest->color().red(),
                           nest->color().green(),
                           nest->color().blue());
    }
  } /* for(*nest..) */

  /*
   * Blocks are inside caches, so display the cache the point is inside FIRST,
   * so that you don't have blocks rendering inside of caches.
   */
  for (auto* cache : arena_map()->caches()) {
    if (cache->contains_point2D(tmp)) {
      return argos::CColor(cache->color().red(),
                           cache->color().green(),
                           cache->color().blue());
    }
  } /* for(&cache..) */

  for (auto* block : arena_map()->blocks()) {
    /*
     * Short circuiting tests for out of sight blocks can help in large
     * swarms with large #s of blocks.
     */
    if (block->is_out_of_sight()) {
      continue;
    }

    /*
     * Even though each block type has a unique color, the only distinction
     * that robots can make to determine if they are on a block or not is
     * between shades of black/white. So, all blocks must appear as black, even
     * when they are not actually (when blocks are picked up their correct color
     * is shown through visualization).
     */
    if (block->contains_point2D(tmp)) {
      return argos::CColor::BLACK;
    }
  } /* for(&block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void depth2_loop_functions::robot_pre_step(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::foraging_controller*>(
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
            "Controller '%s' type '%s' not in depth2 LOS update map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto applicator = ccops::applicator<controller::foraging_controller,
                                      ccops::robot_los_update,
                                      rds::grid2D_overlay<cds::cell2D>,
                                      repr::forager_los>(controller);
  boost::apply_visitor(applicator,
                       m_los_update_map->at(controller->type_index()));
} /* robot_pre_step() */

void depth2_loop_functions::robot_post_step(argos::CFootBotEntity& robot) {
  auto controller = dynamic_cast<controller::foraging_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Watch the robot interact with its environment after physics have been
   * updated and its controller has run.
   *
   * If said interaction results in a block being dropped in a new cache, then
   * we need to re-run dynamic cache creation.
   */
  auto it = m_interactor_map->find(controller->type_index());
  ER_ASSERT(m_interactor_map->end() != it,
            "Controller '%s' type '%s' not in depth2 LOS update map",
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
  if (interactor_status::ekNO_EVENT != status) {
    /*
     * Signal that dynamic cache creation needs to be run AFTER all robots have
     * finished their control steps.
     */
    if (interactor_status::ekNEW_CACHE_BLOCK_DROP & status) {
      m_dynamic_cache_mtx.lock();
      m_dynamic_cache_create = true;
      m_dynamic_cache_mtx.unlock();
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
    if (nullptr != oracle()) {
      oracle()->update(arena_map());
    }
  }

  /* get stats from this robot before its state changes */
  auto mapplicator = ccops::applicator<controller::foraging_controller,
                                       ccops::metrics_extract,
                                       depth2_metrics_aggregator>(controller);

  auto it2 = m_metric_extractor_map->find(controller->type_index());
  ER_ASSERT(m_metric_extractor_map->end() != it2,
            "Controller '%s' type '%s' not in depth2 LOS update map",
            controller->GetId().c_str(),
            controller->type_index().name());

  boost::apply_visitor(mapplicator,
                       m_metric_extractor_map->at(controller->type_index()));
  controller->block_manip_recorder()->reset();
} /* robot_post_step() */

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
  cache_create_ro_params ccp = {
      .current_caches = arena_map()->caches(),
      .clusters = arena_map()->block_distributor()->block_clusters(),
      .t = rtypes::timestep(GetSpace().GetSimulationClock())};

  if (auto created = m_cache_manager->create(ccp, arena_map()->blocks())) {
    arena_map()->caches_add(*created, this);
    floor()->SetChanged();
    return true;
  }
  return false;
} /* cache_creation_handle() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_LOOP_FUNCTIONS(depth2_loop_functions, "depth2_loop_functions");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth2, support, fordyca);
