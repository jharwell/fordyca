/**
 * \file d0_robot_manager.cpp
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
#include "fordyca/ros/support/d0/d0_robot_manager.hpp"

#include <boost/mpl/for_each.hpp>

#include "cosm/controller/operations/applicator.hpp"
#include "cosm/interactors/applicator.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/metrics/specs.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/ros/metrics/d0/d0_robot_metrics_manager.hpp"
#include "fordyca/ros/support/d0/robot_arena_interactor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, ros, support, d0);

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
  RCPPSW_COLD functor_maps_initializer(d0_robot_manager* const lf_in)

      : lf(lf_in) {}
  template <typename T>
  RCPPSW_COLD void operator()(const T& controller) const {
    lf->m_interactor_map->emplace(
        typeid(controller),
        robot_arena_interactor<T>(lf->m_metrics_manager.get()));
    lf->m_metrics_map->emplace(typeid(controller),
                               ccops::metrics_extract<T,
                               frmetrics::d0::d0_robot_metrics_manager>(
                                   lf->m_metrics_manager.get()));
  }

  /* clang-format off */
  d0_robot_manager * const lf;
  /* clang-format on */
};

NS_END(detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
d0_robot_manager::d0_robot_manager(const cros::topic& robot_ns,
                                   const cros::config::sierra_config* config,
                                   fcontroller::foraging_controller* c)
    : ER_CLIENT_INIT("fordyca.ros.support.d0_robot_manager"),
      robot_manager(config),
      mc_robot_ns(robot_ns),
      m_controller(c),
      m_metrics_manager(nullptr),
      m_interactor_map(nullptr),
      m_metrics_map(nullptr) {}

d0_robot_manager::~d0_robot_manager(void) = default;

/*******************************************************************************
 * Initialization
 ******************************************************************************/
void d0_robot_manager::init(ticpp::Element& node) {
  mdc_ts_update();
  ndc_uuid_push();
  ER_INFO("Initializing...");

  shared_init(node);
  private_init();

  ER_INFO("Initialization finished");
  ndc_uuid_pop();
} /* init() */

void d0_robot_manager::shared_init(ticpp::Element& node) {
  robot_manager::init(node);
} /* shared_init() */

void d0_robot_manager::private_init(void) {
  /* initialize output and metrics collection */
  const auto* output = config()->config_get<cpconfig::output_config>();
  m_metrics_manager = std::make_unique<frmetrics::d0::d0_robot_metrics_manager>(
      mc_robot_ns, &output->metrics);

   m_interactor_map = std::make_unique<interactor_map_type>();
   m_metrics_map = std::make_unique<metric_extraction_map_type>();

   /*
   * Intitialize controller interactions with environment via various
   * functors/type maps for all d0 controller types.
   */
  detail::functor_maps_initializer f_initializer(this);
  boost::mpl::for_each<fcontroller::d0::reactive_typelist>(f_initializer);
} /* private_init() */

void d0_robot_manager::pre_step(void) {
  ndc_uuid_push();
  robot_manager::pre_step();
  ndc_uuid_pop();

  /* Update robot position, time */
  robot_pre_step();
} /* pre_step() */

void d0_robot_manager::post_step(void) {
  ndc_uuid_push();
  robot_manager::post_step();
  ndc_uuid_pop();

  /*  Collect metrics from robot */
  robot_post_step();

  ndc_uuid_push();

  /* all metrics collected--send to ROS master node for processing */
  if (m_metrics_manager->flush(rmetrics::output_mode::ekSTREAM, timestep())) {
    ER_DEBUG("Flushed metrics to ROS master");
  }

  m_metrics_manager->interval_reset(timestep());

  ndc_uuid_pop();
} /* post_step() */

void d0_robot_manager::destroy(void) {
  if (nullptr != m_metrics_manager) {
    m_metrics_manager->finalize();
  }
} /* destroy() */

void d0_robot_manager::reset(void) {
  ndc_uuid_push();
  robot_manager::reset();
  m_metrics_manager->initialize();
  ndc_uuid_pop();
} /* reset() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void d0_robot_manager::robot_post_step(void) {
  /*
   * After the robot has run its controller, it "interacts" with its environment
   * via events. This step exists so that the same code can be used for
   * simulations where you have total control of what happens (e.g., ARGoS),
   * those which you might not have total control (e.g., Gazebo), and for real
   * robots where you have zero control.
   */
  auto id = m_controller->type_index();
  auto it = m_interactor_map->find(id);
  ER_ASSERT(m_interactor_map->end() != it,
            "Controller type '%s' not in d0 interactor map",
            id.name());

  auto iapplicator = cinteractors::applicator<controller::foraging_controller,
                                              d0::robot_arena_interactor>(
                                                  m_controller,
                                                  timestep());
  boost::apply_visitor(iapplicator, m_interactor_map->at(id));

  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto mapplicator = ccops::applicator<
    controller::foraging_controller,
    ccops::metrics_extract,
    frmetrics::d0::d0_robot_metrics_manager
    >(m_controller);

  boost::apply_visitor(mapplicator, m_metrics_map->at(id));
  m_controller->block_manip_recorder()->reset();
} /* robot_post_step() */

void d0_robot_manager::robot_pre_step(void) {
  /*
   * Unless and until I need to handle more complex controllers than CRW,
   * cheating and just setting the resolution to something reasonable for the
   * purpose of metric collection I think is fine.
   */
  m_controller->sensing_update(timestep(),
                               rtypes::discretize_ratio(0.2));
} /* robot_pre_step() */

NS_END(d0, support, ros, fordyca);
