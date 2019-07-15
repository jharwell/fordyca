/**
 * @file depth0_loop_functions.cpp
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
#include "fordyca/support/depth0/depth0_loop_functions.hpp"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <boost/mpl/for_each.hpp>

#include "fordyca/config/arena/arena_map_config.hpp"
#include "fordyca/config/output_config.hpp"
#include "fordyca/config/visualization_config.hpp"
#include "fordyca/controller/depth0/crw_controller.hpp"
#include "fordyca/controller/depth0/dpo_controller.hpp"
#include "fordyca/controller/depth0/mdpo_controller.hpp"
#include "fordyca/controller/depth0/odpo_controller.hpp"
#include "fordyca/controller/depth0/omdpo_controller.hpp"
#include "fordyca/metrics/blocks/transport_metrics_collector.hpp"
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"
#include "fordyca/support/depth0/robot_arena_interactor.hpp"
#include "fordyca/support/depth0/robot_configurer.hpp"
#include "fordyca/support/depth0/robot_configurer_adaptor.hpp"
#include "fordyca/support/depth0/robot_los_updater_adaptor.hpp"
#include "fordyca/support/oracle/oracle_manager.hpp"
#include "fordyca/support/robot_interactor_adaptor.hpp"
#include "fordyca/support/robot_los_updater_adaptor.hpp"
#include "fordyca/support/robot_metric_extractor.hpp"
#include "fordyca/support/robot_metric_extractor_adaptor.hpp"
#include "fordyca/support/swarm_iterator.hpp"
#include "fordyca/repr/line_of_sight.hpp"

#include "rcppsw/swarm/convergence/convergence_calculator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, depth0);
using ds::arena_grid;

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
NS_START(detail);

/**
 * @struct functor_maps_initializer
 * @ingroup fordyca support depth0 detail
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
        robot_arena_interactor<T>(lf->arena_map(),
                                  lf->m_metrics_agg.get(),
                                  lf->floor(),
                                  lf->tv_manager()));
    lf->m_metrics_map->emplace(
        typeid(controller),
        robot_metric_extractor<depth0_metrics_aggregator, T>(
            lf->m_metrics_agg.get()));
    config_map->emplace(
        typeid(controller),
        robot_configurer<T>(
            lf->config()->config_get<config::visualization_config>(),
            nullptr != lf->oracle_manager()
                ? lf->oracle_manager()->entities_oracle()
                : nullptr));
    lf->m_los_update_map->emplace(typeid(controller),
                                  robot_los_updater<T>(lf->arena_map()));
  }

  /* clang-format off */
  depth0_loop_functions * const      lf;
  detail::configurer_map_type* const config_map;
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
void depth0_loop_functions::Init(ticpp::Element& node) {
  ndc_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_pop();
} /* Init() */

void depth0_loop_functions::shared_init(ticpp::Element& node) {
  base_loop_functions::Init(node);
} /* shared_init() */

void depth0_loop_functions::private_init(void) {
  /* initialize output and metrics collection */
  auto* arena = config()->config_get<config::arena::arena_map_config>();
  config::output_config output = *config()->config_get<config::output_config>();
  output.metrics.arena_grid = arena->grid;

  m_metrics_agg = std::make_unique<depth0_metrics_aggregator>(&output.metrics,
                                                              output_root());
  m_interactor_map = std::make_unique<interactor_map_type>();
  m_metrics_map = std::make_unique<metric_extraction_map_type>();
  m_los_update_map = std::make_unique<detail::los_updater_map_type>();

  /* only needed for initialization, so not a member */
  auto config_map = detail::configurer_map_type();

  /*
   * Intitialize robot interactions with environment via various functors/type
   * maps.
   */
  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::depth0::typelist>(f_initializer);

  /* configure robots */
  swarm_iterator::controllers(this, [&](auto* controller) {
    boost::apply_visitor(detail::robot_configurer_adaptor(controller),
                         config_map.at(controller->type_index()));
  });
} /* private_init() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void depth0_loop_functions::robot_timestep_process(argos::CFootBotEntity& robot) {
  auto controller = static_cast<controller::base_controller*>(
      &robot.GetControllableEntity().GetController());

  /* collect metrics from robot before its state changes */
  auto madaptor =
      robot_metric_extractor_adaptor<depth0_metrics_aggregator>(controller);
  boost::apply_visitor(madaptor, m_metrics_map->at(controller->type_index()));

  controller->block_manip_collator()->reset();

  /* Set robot position, time, and send it its new LOS */
  utils::set_robot_pos<decltype(*controller)>(
      robot, arena_map()->grid_resolution());
  utils::set_robot_tick<decltype(*controller)>(
      robot, rtypes::timestep(GetSpace().GetSimulationClock()));
  boost::apply_visitor(detail::robot_los_updater_adaptor(controller),
                       m_los_update_map->at(controller->type_index()));

  /* Watch the robot interact with the environment! */
  auto iadaptor =
      robot_interactor_adaptor<depth0::robot_arena_interactor, interactor_status>(
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
   * timestep. See #577.
   */
  if (interactor_status::ekNoEvent != status && nullptr != oracle_manager()) {
    oracle_manager()->update(arena_map());
  }
} /* robot_timestep_process() */

argos::CColor depth0_loop_functions::GetFloorColor(
    const argos::CVector2& plane_pos) {
  rmath::vector2d tmp(plane_pos.GetX(), plane_pos.GetY());
  if (arena_map()->nest().contains_point(tmp)) {
    return argos::CColor(arena_map()->nest().color().red(),
                         arena_map()->nest().color().green(),
                         arena_map()->nest().color().blue());
  }

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
  } /* for(block..) */

  return argos::CColor::WHITE;
} /* GetFloorColor() */

void depth0_loop_functions::PreStep(void) {
  ndc_push();
  base_loop_functions::PreStep();

  auto& collector = static_cast<metrics::blocks::transport_metrics_collector&>(
      *(*m_metrics_agg)["blocks::transport"]);
  arena_map()->redist_governor()->update(
      rtypes::timestep(GetSpace().GetSimulationClock()),
      collector.cum_collected(),
      nullptr != conv_calculator() ? conv_calculator()->converged() : false);

  /* Process all robots */
  swarm_iterator::robots(this,
                         [&](auto* robot) { robot_timestep_process(*robot); });

  /* collect metrics from non-robot sources */
  m_metrics_agg->collect_from_loop(this);

  /* Not a clean way to do this in the convergence metrics collector... */
  if (m_metrics_agg->metrics_write_all(
          rtypes::timestep(GetSpace().GetSimulationClock())) &&
      nullptr != conv_calculator()) {
    conv_calculator()->reset_metrics();
  }
  /* write out all metrics */
  m_metrics_agg->timestep_inc_all();
  m_metrics_agg->interval_reset_all();

  ndc_pop();
} /* PreStep() */

void depth0_loop_functions::Destroy(void) {
  if (nullptr != m_metrics_agg) {
    m_metrics_agg->finalize_all();
  }
} /* Destroy() */

void depth0_loop_functions::Reset(void) {
  ndc_push();
  base_loop_functions::Reset();
  m_metrics_agg->reset_all();
  ndc_pop();
} /* Reset() */

using namespace argos; // NOLINT
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#pragma clang diagnostic ignored "-Wglobal-constructors"
REGISTER_LOOP_FUNCTIONS(depth0_loop_functions, "depth0_loop_functions");
#pragma clang diagnostic pop;
NS_END(depth0, support, fordyca);
