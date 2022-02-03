/**
 * \file nest_block_drop_interactor.hpp
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

#ifndef INCLUDE_FORDYCA_ROS_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_ROS_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "cosm/ros/interactors/nest_block_process.hpp"

#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "fordyca/metrics/blocks/block_manip_events.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class nest_block_drop_interactor
 * \ingroup ros support
 *
 * \brief Handle's a robot's (possible) \ref nest_block_drop event on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class nest_block_drop_interactor final
    : public crinteractors::nest_block_process<TController, TControllerSpecMap> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;

  using robot_metrics_manager_type = typename controller_spec::robot_metrics_manager_type;
  using interactor_status_type = typename controller_spec::interactor_status_type;
  using robot_nest_block_process_visitor_type =
      typename controller_spec::robot_nest_block_process_visitor_type;

  nest_block_drop_interactor(robot_metrics_manager_type* const metrics)
      : crinteractors::nest_block_process<TController,
                                          TControllerSpecMap>(metrics) {}

  nest_block_drop_interactor(nest_block_drop_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  nest_block_drop_interactor(const nest_block_drop_interactor&) = delete;
  nest_block_drop_interactor&
  operator=(const nest_block_drop_interactor&) = delete;

  bool robot_goal_acquired(const TController& controller) const override {
    return controller.goal_acquired() && fsm::foraging_transport_goal::ekNEST ==
        controller.block_transport_goal();
  }

  void robot_previsit_hook(TController& controller) const override {
    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     *
     * We set the penalty duration to 0 for now for simplicity with real
     * robots. This may be revisited later.
     */
    controller.block_manip_recorder()->record(
        fmetrics::blocks::block_manip_events::ekFREE_DROP,
        rtypes::timestep(0));
  }
};

NS_END(support, ros, fordyca);

#endif /* INCLUDE_FORDYCA_ROS_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_ */
