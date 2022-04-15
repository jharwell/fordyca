/**
 * \file free_block_pickup_interactor.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/ros/interactors/free_block_pickup.hpp"

#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/metrics/blocks/block_manip_events.hpp"
#include "fordyca/support/interactor_status.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class free_block_pickup_interactor
 * \ingroup ros support
 *
 * \brief Handle's a robot's (possible) free block pickup event on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class free_block_pickup_interactor final
    : public crinteractors::free_block_pickup<TController, TControllerSpecMap> {
 public:
  using controller_spec =
      typename boost::mpl::at<TControllerSpecMap, TController>::type;
  using robot_block_pickup_visitor_type =
      typename controller_spec::robot_block_pickup_visitor_type;

  free_block_pickup_interactor(void) = default;

  free_block_pickup_interactor(free_block_pickup_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  free_block_pickup_interactor(const free_block_pickup_interactor&) = delete;
  free_block_pickup_interactor&
  operator=(const free_block_pickup_interactor&) = delete;

 private:
  bool robot_goal_acquired(const TController& controller) const override {
    return controller.goal_acquired() &&
        fsm::foraging_acq_goal::ekBLOCK == controller.acquisition_goal();
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
        fmetrics::blocks::block_manip_events::ekFREE_PICKUP,
        rtypes::timestep(0));
  }
};

NS_END(support, ros, fordyca);
