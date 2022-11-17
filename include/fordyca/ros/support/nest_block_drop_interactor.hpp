/**
 * \file nest_block_drop_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ros/interactors/nest_block_process.hpp"
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"

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
    return controller.goal_acquired() &&
        fsm::foraging_transport_goal::ekNEST ==
        controller.block_transport_goal();
  }

  void robot_previsit_hook(TController& controller) const override {
    auto penalty = rtypes::timestep(0);

    /*
     * If the block drop strategy is defined, then obviously it takes that long
     * to drop a block, so we use that as our drop "penalty".
     */
    if (auto strategy = controller.block_drop_strategy()) {
      penalty = strategy->config()->duration;
    }

    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     */
    controller.block_manip_recorder()->record(
        fmetrics::blocks::block_manip_events::ekFREE_DROP,
                                              penalty);
  }
};

NS_END(support, ros, fordyca);
