/**
 * \file depth0_loop_functions.cpp
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
#include "fordyca/support/depth0/depth0_loop_functions.hpp"

#include <boost/mpl/for_each.hpp>

#include "cosm/arena/config/arena_map_config.hpp"
#include "cosm/controller/operations/applicator.hpp"
#include "cosm/foraging/oracle/foraging_oracle.hpp"
#include "cosm/metrics/blocks/transport_metrics_collector.hpp"
#include "cosm/operations/robot_arena_interaction_applicator.hpp"
#include "cosm/pal/argos_convergence_calculator.hpp"
#include "cosm/pal/argos_swarm_iterator.hpp"

#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth0/odpo_controller.hpp"
#include "fordyca/controller/depth0/omdpo_controller.hpp"
#include "fordyca/controller/foraging_perception_subsystem.hpp"
#include "fordyca/repr/forager_los.hpp"
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"
#include "fordyca/support/depth0/robot_arena_interactor.hpp"
#include "fordyca/support/depth0/robot_configurer.hpp"
#include "fordyca/support/depth0/robot_configurer_applicator.hpp"
#include "fordyca/support/depth0/robot_los_update_applicator.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

/**
 * \struct functor_maps_initializer
 * \ingroup support depth0 detail
 *
 * Convenience class containing initialization for all of the typeid ->
 * boost::variant maps for all controller types that are used throughout
 * initialization and simulation.
 */
struct functor_maps_initializer {
  RCSW_COLD functor_maps_initializer(configurer_map_type* const cmap,
                                     depth0_loop_functions* const lf_in)

      : lf(lf_in), config_map(cmap) {}
  template <typename T>
  RCSW_COLD void operator()(const T& controller) const {
    lf->m_interactor_map->emplace(
        typeid(controller),
        robot_arena_interactor<T, carena::caching_arena_map>(
            lf->arena_map(),
            lf->m_metrics_agg.get(),
            lf->floor(),
            lf->tv_manager()->dynamics<ctv::dynamics_type::ekENVIRONMENT>()));
    lf->m_metrics_map->emplace(
        typeid(controller),
        ccops::metrics_extract<T, depth0_metrics_aggregator>(
            lf->m_metrics_agg.get()));
    config_map->emplace(
        typeid(controller),
        robot_configurer<T>(
            lf->config()->config_get<cvconfig::visualization_config>(),
            lf->oracle()));
    lf->m_los_update_map->emplace(
        typeid(controller),
        ccops::robot_los_update<T,
                                rds::grid2D_overlay<cds::cell2D>,
                                repr::forager_los>(
            lf->arena_map()->decoratee().template layer<cds::arena_grid::kCell>()));
  }

  /* clang-format off */
  depth0_loop_functions * const lf;
  configurer_map_type* const    config_map;
  /* clang-format on */
};

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
depth0_loop_functions::depth0_loop_functions(void)
    : ER_CLIENT_INIT("fordyca.loop.depth0"),
      m_metrics_agg(nullptr),
      m_interactor_map(nullptr),
      m_metrics_map(nullptr),
      m_los_update_map(nullptr) {}

depth0_loop_functions::~depth0_loop_functions(void) = default;

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/
void depth0_loop_functions::init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* init() */

void depth0_loop_functions::shared_init(ticpp::Element& node) {
  base_loop_functions::init(node);
} /* shared_init() */

void depth0_loop_functions::private_init(void) {
  /* initialize output and metrics collection */
  auto* output = config()->config_get<cmconfig::output_config>();

  /*
   * Need to give spatial metrics collectors the padded arena size in order to
   * avoid boost::assert failures when robots are near the upper edge of the
   * arena map. The arena map pads the size obtained from the XML file after
   * initialization, so we just need to grab it.
   */
  auto padded_size =
      rmath::vector2d(arena_map()->xrsize(), arena_map()->yrsize());
  auto arena = *config()->config_get<caconfig::arena_map_config>();
  arena.grid.dims = padded_size;
  m_metrics_agg = std::make_unique<depth0_metrics_aggregator>(&output->metrics,
                                                              &arena.grid,
                                                              output_root());
  /* this starts at 0, and ARGoS starts at 1, so sync up */
  m_metrics_agg->timestep_inc_all();

  m_interactor_map = std::make_unique<interactor_map_type>();
  m_los_update_map = std::make_unique<los_updater_map_type>();
  m_metrics_map = std::make_unique<metric_extraction_map_type>();

  /* only needed for initialization, so not a member */
  auto config_map = configurer_map_type();

  /*
   * Intitialize controller interactions with environment via various
   * functors/type maps for all depth0 controller types.
   */
  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::depth0::typelist>(f_initializer);

  /* configure robots */
  auto cb = [&](auto* controller) {
    ER_ASSERT(config_map.end() != config_map.find(controller->type_index()),
              "Controller '%s' type '%s' not in depth0 configuration map",
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
  cpal::argos_swarm_iterator::controllers<argos::CFootBotEntity,
                                          controller::foraging_controller,
                                          cpal::iteration_order::ekSTATIC>(
      this, cb, kARGoSRobotType);
} /* private_init() */

/*******************************************************************************
 * ARGoS Hooks
 ******************************************************************************/
void depth0_loop_functions::pre_step(void) {
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

void depth0_loop_functions::post_step(void) {
  ndc_push();
  base_loop_functions::post_step();
  ndc_pop();

  /* Process all robots: interact with environment then collect metrics */
  auto cb = [&](argos::CControllableEntity* robot) {
    ndc_push();
    robot_post_step(dynamic_cast<argos::CFootBotEntity&>(robot->GetParent()));
    ndc_pop();
  };
  cpal::argos_swarm_iterator::robots<cpal::iteration_order::ekDYNAMIC>(this, cb);

  ndc_push();
  /* Update block distribution status */
  auto* collector =
      m_metrics_agg->get<cmetrics::blocks::transport_metrics_collector>(
          "blocks::transport");
  arena_map()->redist_governor()->update(
      rtypes::timestep(GetSpace().GetSimulationClock()),
      collector->cum_transported(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);

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

void depth0_loop_functions::destroy(void) {
  if (nullptr != m_metrics_agg) {
    m_metrics_agg->finalize_all();
  }
} /* destroy() */

void depth0_loop_functions::reset(void) {
  ndc_push();
  base_loop_functions::reset();
  m_metrics_agg->reset_all();
  ndc_pop();
} /* reset() */

argos::CColor depth0_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  rmath::vector2d tmp(plane_pos.GetX(), plane_pos.GetY());
  if (arena_map()->nest().contains_point(tmp)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }

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
  } /* for(block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void depth0_loop_functions::robot_pre_step(argos::CFootBotEntity& robot) {
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
            "Controller '%s' type '%s' not in depth0 LOS update map",
            controller->GetId().c_str(),
            controller->type_index().name());
  auto applicator = robot_los_update_applicator(controller);
  boost::apply_visitor(applicator,
                       m_los_update_map->at(controller->type_index()));
} /* robot_pre_step() */

void depth0_loop_functions::robot_post_step(argos::CFootBotEntity& robot) {
  auto controller = static_cast<controller::foraging_controller*>(
      &robot.GetControllableEntity().GetController());
  /*
   * Watch the robot interact with its environment after physics have been
   * updated and its controller has run.
   */
  auto it = m_interactor_map->find(controller->type_index());
  ER_ASSERT(m_interactor_map->end() != it,
            "Controller '%s' type '%s' not in depth0 interactor map",
            controller->GetId().c_str(),
            controller->type_index().name());

  auto iapplicator =
      cops::robot_arena_interaction_applicator<controller::foraging_controller,
                                               depth0::robot_arena_interactor,
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
   * ignored/compiled out, which is not a problem, because the LOS processing
   * errors that can result are transient and are corrected the next
   * timestep. See #577.
   */
  if (interactor_status::ekNO_EVENT != status && nullptr != oracle()) {
    oracle()->update(arena_map());
  }
  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto mapplicator = ccops::applicator<controller::foraging_controller,
                                       ccops::metrics_extract,
                                       depth0_metrics_aggregator>(controller);
  boost::apply_visitor(mapplicator, m_metrics_map->at(controller->type_index()));

  controller->block_manip_recorder()->reset();
} /* robot_post_step() */

using namespace argos; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_LOOP_FUNCTIONS(depth0_loop_functions, "depth0_loop_functions");

RCPPSW_WARNING_DISABLE_POP()

NS_END(depth0, support, fordyca);
