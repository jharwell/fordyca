/**
 * \file d0_loop_functions.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/d0/d0_loop_functions.hpp"

#include <boost/mpl/for_each.hpp>

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/argos/convergence_calculator.hpp"
#include "cosm/controller/operations/applicator.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics_collector.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"
#include "cosm/interactors/applicator.hpp"
#include "cosm/metrics/specs.hpp"
#include "cosm/pal/argos/swarm_iterator.hpp"
#include "cosm/pal/pal.hpp"

#include "fordyca/argos/metrics/d0/d0_metrics_manager.hpp"
#include "fordyca/argos/support/d0/robot_arena_interactor.hpp"
#include "fordyca/argos/support/d0/robot_configurer.hpp"
#include "fordyca/argos/support/d0/robot_configurer_applicator.hpp"
#include "fordyca/argos/support/d0/robot_los_update_applicator.hpp"
#include "fordyca/argos/support/tv/env_dynamics.hpp"
#include "fordyca/argos/support/tv/fordyca_pd_adaptor.hpp"
#include "fordyca/argos/support/tv/tv_manager.hpp"
#include "fordyca/controller/cognitive/d0/dpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/mdpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/odpo_controller.hpp"
#include "fordyca/controller/cognitive/d0/omdpo_controller.hpp"
#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/repr/forager_los.hpp"
#include "fordyca/subsystem/perception/foraging_perception_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, d0);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

/**
 * \struct functor_maps_initializer
 * \ingroup support d0 detail
 *
 * Convenience class containing initialization for all of the typeid ->
 * boost::variant maps for all controller types that are used throughout
 * initialization and simulation.
 */
struct functor_maps_initializer {
  RCPPSW_COLD functor_maps_initializer(configurer_map_type* const cmap,
                                       d0_loop_functions* const lf_in)

      : lf(lf_in), config_map(cmap) {}
  template <typename T>
  RCPPSW_COLD void operator()(const T& controller) const {
    lf->m_interactor_map->emplace(
        typeid(controller),
        robot_arena_interactor<T, carena::caching_arena_map>(
            lf->arena_map(),
            lf->m_metrics_manager.get(),
            lf->floor(),
            lf->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>()));
    lf->m_metrics_map->emplace(
        typeid(controller),
        ccops::metrics_extract<T, fametrics::d0::d0_metrics_manager>(
            lf->m_metrics_manager.get()));
    config_map->emplace(
        typeid(controller),
        robot_configurer<T>(
            lf->config()->config_get<cavis::config::visualization_config>(),
            lf->oracle()));
    lf->m_los_update_map->emplace(
        typeid(controller),
        ccops::grid_los_update<T,
                               rds::grid2D_overlay<cds::cell2D>,
                               repr::forager_los>(
            lf->arena_map()
                ->decoratee()
                .template layer<cads::arena_grid::kCell>()));
  }

  /* clang-format off */
  d0_loop_functions * const lf;
  configurer_map_type* const    config_map;
  /* clang-format on */
};

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
d0_loop_functions::d0_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.argos.loop.d0"),
      m_metrics_manager(nullptr),
      m_interactor_map(nullptr),
      m_metrics_map(nullptr),
      m_los_update_map(nullptr) {}

d0_loop_functions::~d0_loop_functions(void) = default;

/*******************************************************************************
 * Initialization
 ******************************************************************************/
void d0_loop_functions::init(ticpp::Element& node) {
  mdc_ts_update();
  ndc_uuid_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void d0_loop_functions::shared_init(ticpp::Element& node) {
  argos_swarm_manager::init(node);
} /* shared_init() */

void d0_loop_functions::private_init(void) {
  /* initialize output and metrics collection */
  const auto* output = config()->config_get<cpconfig::output_config>();
  const auto* arena = config()->config_get<caconfig::arena_map_config>();
  m_metrics_manager = std::make_unique<fametrics::d0::d0_metrics_manager>(
      &output->metrics,
      &arena->grid,
      output_root(),
      arena_map()->block_distributor()->block_clustersro().size());

  m_interactor_map = std::make_unique<interactor_map_type>();
  m_los_update_map = std::make_unique<los_updater_map_type>();
  m_metrics_map = std::make_unique<metric_extraction_map_type>();

  /* only needed for initialization, so not a member */
  auto config_map = configurer_map_type();

  /*
   * Intitialize controller interactions with environment via various
   * functors/type maps for all d0 controller types.
   */
  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::d0::typelist>(f_initializer);

  /* configure robots */
  auto cb = [&](auto* controller) {
    ER_ASSERT(config_map.end() != config_map.find(controller->type_index()),
              "Controller '%s' type '%s' not in d0 configuration map",
              controller->GetId().c_str(),
              controller->type_index().name());

    auto applicator = robot_configurer_applicator(controller);
    boost::apply_visitor(applicator, config_map.at(controller->type_index()));
  };

  /*
   * Even though this CAN be done in dynamic order, during initialization ARGoS
   * threads are not set up yet so doing dynamicaly causes a deadlock. Also, it
   * only happens once, so it doesn't really matter if it is slow.
   */
  cpargos::swarm_iterator::controllers<controller::foraging_controller,
                                       cpal::iteration_order::ekSTATIC>(
      this, cb, cpal::kRobotType);
} /* private_init() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void d0_loop_functions::pre_step(void) {
  mdc_ts_update();
  ndc_uuid_push();
  argos_swarm_manager::pre_step();
  ndc_uuid_pop();

  /* Process all robots */
  auto cb = [&](::argos::CControllableEntity* robot) {
    ndc_uuid_push();
    robot_pre_step(dynamic_cast<chal::robot&>(robot->GetParent()));
    ndc_uuid_pop();
  };
  cpargos::swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);
} /* pre_step() */

void d0_loop_functions::post_step(void) {
  ndc_uuid_push();
  argos_swarm_manager::post_step();
  ndc_uuid_pop();

  /* Process all robots: interact with environment then collect metrics */
  auto cb = [&](::argos::CControllableEntity* robot) {
    ndc_uuid_push();
    robot_post_step(dynamic_cast<chal::robot&>(robot->GetParent()));
    ndc_uuid_pop();
  };
  cpargos::swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);

  ndc_uuid_push();

  const auto* collector =
      m_metrics_manager->get<cfmetrics::block_transportee_metrics_collector>(
          cmspecs::blocks::kTransportee.scoped());

  /* update arena map */
  arena_map()->post_step_update(
      timestep(),
      collector->cum_transported(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);

  /* Collect metrics from loop functions */
  m_metrics_manager->collect_from_sm(this);

  m_metrics_manager->flush(rmetrics::output_mode::ekTRUNCATE, timestep());
  m_metrics_manager->flush(rmetrics::output_mode::ekCREATE, timestep());

  /* Not a clean way to do this in the metrics collectors... */
  if (m_metrics_manager->flush(rmetrics::output_mode::ekAPPEND, timestep())) {
    if (nullptr != conv_calculator()) {
      conv_calculator()->reset_metrics();
    }
    tv_manager()->dynamics<ctv::dynamics_type::ekPOPULATION>()->reset_metrics();
  }
  m_metrics_manager->interval_reset(timestep());

  ndc_uuid_pop();
} /* post_step() */

void d0_loop_functions::destroy(void) {
  if (nullptr != m_metrics_manager) {
    m_metrics_manager->finalize();
  }
} /* destroy() */

void d0_loop_functions::reset(void) {
  ndc_uuid_push();
  argos_swarm_manager::reset();
  m_metrics_manager->initialize();
  ndc_uuid_pop();
} /* reset() */

/*******************************************************************************
 * General Member
 ******************************************************************************/
void d0_loop_functions::robot_pre_step(chal::robot& robot) {
  auto* controller = static_cast<controller::foraging_controller*>(
      &robot.GetControllableEntity().GetController());

  /*
   * Update robot position, time. This can't be done as part of the robot's
   * control step because we need access to information only available in the
   * loop functions.
   */
  controller->sensing_update(timestep(), arena_map()->grid_resolution());

  /* Send robot its new LOS */
  auto it = m_los_update_map->find(controller->type_index());
  ER_ASSERT(m_los_update_map->end() != it,
            "Controller '%s' type '%s' not in d0 LOS update map",
            controller->GetId().c_str(),
            controller->type_index().name());
  auto applicator = robot_los_update_applicator(controller);
  boost::apply_visitor(applicator,
                       m_los_update_map->at(controller->type_index()));
} /* robot_pre_step() */

void d0_loop_functions::robot_post_step(chal::robot& robot) {
  auto* controller = static_cast<controller::foraging_controller*>(
      &robot.GetControllableEntity().GetController());
  /*
   * Watch the robot interact with its environment after physics have been
   * updated and its controller has run.
   */
  auto it = m_interactor_map->find(controller->type_index());
  ER_ASSERT(m_interactor_map->end() != it,
            "Controller '%s' type '%s' not in d0 interactor map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto iapplicator =
      cinteractors::applicator<controller::foraging_controller,
                               d0::robot_arena_interactor,
                               carena::caching_arena_map>(controller, timestep());
  auto status = boost::apply_visitor(
      iapplicator, m_interactor_map->at(controller->type_index()));

  /*
   * The oracle does not necessarily have up-to-date information about all
   * blocks in the arena, as a robot could have dropped a block in the nest or
   * picked one up, so its version of the set of free blocks in the arena is out
   * of date. Robots processed *after* the robot that caused the event need
   * the correct free block set to be available from the oracle upon request, to
   * avoid asserts during on debug builds. On optimized builds the asserts are
   * ignored/compiled out, which is not a problem, because the LOS processing
   * errors that can result are transient and are corrected the next
   * timestep. See FORDYCA#577.
   */
  if (fsupport::interactor_status::ekNO_EVENT != status && nullptr != oracle()) {
    oracle()->update(arena_map());
  }

  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto mapplicator =
      ccops::applicator<controller::foraging_controller,
                        ccops::metrics_extract,
                        fametrics::d0::d0_metrics_manager>(controller);
  boost::apply_visitor(mapplicator, m_metrics_map->at(controller->type_index()));

  controller->block_manip_recorder()->reset();
} /* robot_post_step() */

NS_END(d0, support, argos, fordyca);

using namespace fasd0; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_LOOP_FUNCTIONS(d0_loop_functions, "d0_loop_functions");

RCPPSW_WARNING_DISABLE_POP()
