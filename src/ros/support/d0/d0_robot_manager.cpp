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
#include "cosm/pal/swarm_iterator.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/metrics/specs.hpp"

#include "fordyca/controller/reactive/d0/crw_controller.hpp"
#include "fordyca/ros/metrics/d0/d0_metrics_manager.hpp"
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
    lf->m_metrics_map->emplace(typeid(controller),
                               ccops::metrics_extract<T,
                               fametrics::d0::d0_metrics_manager>(
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
d0_robot_manager::d0_robot_manager(fcontroller::foraging_controller* c)
    : ER_CLIENT_INIT("fordyca.ros.loop.d0"),
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
  ros_swarm_manager::init(node);
} /* shared_init() */

void d0_robot_manager::private_init(void) {
  /* initialize output and metrics collection */
  const auto* output = config()->config_get<cpconfig::output_config>();
  m_metrics_manager = std::make_unique<fametrics::d0::d0_metrics_manager>(
      &output->metrics,
      output_root());

  /* this starts at 0, and ROS starts at 1, so sync up */
  m_metrics_manager->timestep_inc();

  m_interactor_map = std::make_unique<interactor_map_type>();
  m_metrics_map = std::make_unique<metric_extraction_map_type>();

  /*
   * Intitialize controller interactions with environment via various
   * functors/type maps for all d0 controller types.
   */
  detail::functor_maps_initializer f_initializer(&config_map, this);
  boost::mpl::for_each<controller::d0::typelist>(f_initializer);
} /* private_init() */

void d0_robot_manager::post_step(void) {
  ndc_uuid_push();
  ros_swarm_manager::post_step();
  ndc_uuid_pop();

  /*  Collect metrics */
  auto cb = [&](cros::topic& ns) {
    ndc_uuid_push();
    robot_post_step(ns);
    ndc_uuid_pop();
  };
  cpros::swarm_iterator::robots(this, cb);

  ndc_uuid_push();

  /* Collect metrics from loop functions */
  m_metrics_manager->collect_from_sm(this);

  /* all metrics collected--send to ROS master node for processing */
  m_metrics_manager->flush(rmetrics::output_mode::ekSTREAM);

  m_metrics_manager->interval_reset();
  m_metrics_manager->timestep_inc();

  ndc_uuid_pop();
} /* post_step() */

void d0_robot_manager::destroy(void) {
 if (nullptr != m_metrics_manager) {
    m_metrics_manager->finalize();
  }
} /* destroy() */

void d0_robot_manager::reset(void) {
  ndc_uuid_push();
  ros_swarm_manager::reset();
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
            "Controller '%s' type '%s' not in d0 interactor map",
            controller->GetId().c_str(),
            id.name());

  auto iapplicator = cinteractors::applicator<controller::foraging_controller,
                                              d0::robot_arena_interactor,
                                              carena::caching_arena_map>(
                                                  m_controller,
                                                  timestep());
  auto status = boost::apply_visitor(iapplicator, m_interactor_map->at(id));

  /*
   * Collect metrics from robot, now that it has finished interacting with the
   * environment and no more changes to its state will occur this timestep.
   */
  auto mapplicator = ccops::applicator<controller::foraging_controller,
                                       ccops::metrics_extract,
                                       fametrics::d0::d0_metrics_manager>(controller);
  boost::apply_visitor(mapplicator, m_metrics_map->at(id));
  controller->block_manip_recorder()->reset();
} /* robot_post_step() */

NS_END(d0, support, ros, fordyca);
