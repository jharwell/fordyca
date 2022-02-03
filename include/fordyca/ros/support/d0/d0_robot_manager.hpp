/**
 * \file d0_robot_manager.hpp
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

#ifndef INCLUDE_FORDYCA_ROS_SUPPORT_D0_D0_ROBOT_MANAGER_HPP_
#define INCLUDE_FORDYCA_ROS_SUPPORT_D0_D0_ROBOT_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/mpl/typelist.hpp"

#include "cosm/controller/operations/metrics_extract.hpp"
#include "cosm/hal/robot.hpp"
#include "cosm/ros/topic.hpp"

#include "fordyca/ros/support/foraging_robot_manager.hpp"
#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca::ros::metrics::d0 {
class d0_robot_metrics_manager;
} /* namespace fordyca::ros::metrics::d0 */

NS_START(fordyca, ros, support, d0);

template<typename Controller>
class robot_arena_interactor;

namespace detail {
struct functor_maps_initializer;
} /* namespace detail */

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d0_robot_manager
 * \ingroup ros support d0
 *
 * \brief Contains the simulation support functions for d0 foraging robots
 * running ROS, such as:
 *
 * - Metric collection from robots
 */
class d0_robot_manager : public frsupport::foraging_robot_manager,
                          public rer::client<d0_robot_manager> {
 public:
  d0_robot_manager(fcontroller::foraging_controller* c) RCPPSW_COLD;
  ~d0_robot_manager(void) override RCPPSW_COLD;

  /* swarm manager overrides */
  void init(ticpp::Element& node) override RCPPSW_COLD;
  void pre_step(void) override;
  void post_step(void) override;
  void reset(void) override RCPPSW_COLD;
  void destroy(void) override RCPPSW_COLD;

 protected:
  /**
   * \brief Initialize d0 support to be shared with derived classes:
   *
   * - Depth 0 metric collection
   */
  void shared_init(ticpp::Element& node) RCPPSW_COLD;

private:
  using interactor_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<fcontroller::d0::reactive_typelist,
                              robot_arena_interactor>::type
    >;

  using metric_extraction_map_type = rds::type_map<
    rmpl::typelist_wrap_apply<fcontroller::d0::reactive_typelist,
                              ccops::metrics_extract,
                              frmetrics::d0::d0_robot_metrics_manager>::type>;
  /**
   * \brief These are friend classes because they are basically just pieces of
   * the loop functions pulled out for increased clarity/modularity, and are not
   * meant to be used in other contexts.
   *
   * Doing things this way rather than passing 8 parameters to the functors
   * seemed much cleaner.
   */
  friend detail::functor_maps_initializer;

  /**
   * \brief Initialize d0 support not shared with derived classes:
   *
   * - Various maps mapping controller types to metric collection, controller
   *   initialization.
   */
  void private_init(void) RCPPSW_COLD;

  /**
   * \brief Process a single robot on a timestep, after running its controller.
   *
   * - Collect metrics from it.
   */
  void robot_post_step(void);

  /* clang-format off */
  fcontroller::foraging_controller*                        m_controller;
  std::unique_ptr<frmetrics::d0::d0_robot_metrics_manager> m_metrics_manager;
  std::unique_ptr<interactor_map_type>                     m_interactor_map;
  std::unique_ptr<metric_extraction_map_type>              m_metrics_map;
  /* clang-format on */
};

NS_END(d0, support, ros, fordyca);

#endif /* INCLUDE_FORDYCA_ROS_SUPPORT_D0_D0_ROBOT_MANAGER_HPP_ */
