/**
 * \file d0/robot_arena_interactor.hpp
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
#include "fordyca/ros/support/free_block_pickup_interactor.hpp"
#include "fordyca/ros/support/nest_block_drop_interactor.hpp"
#include "fordyca/ros/support/mpl/free_block_pickup_spec.hpp"
#include "fordyca/ros/support/mpl/nest_block_drop_spec.hpp"

#include "fordyca/controller/reactive/d0/events/nest_block_drop.hpp"
#include "fordyca/controller/reactive/d0/events/free_block_pickup.hpp"
#include "fordyca/controller/reactive/d0/events/block_vanished.hpp"
#include "fordyca/ros/metrics/d0/d0_robot_metrics_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, ros, support, d0);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class robot_arena_interactor
 * \ingroup ros support d0
 *
 * \brief Handle's a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 * - Picking up a free block.
 * - Dropping a carried block in the nest.
 */
template <typename TController>
class robot_arena_interactor final : public rer::client<
  robot_arena_interactor<TController>
  > {
 public:
  explicit robot_arena_interactor(frmetrics::d0::d0_robot_metrics_manager *const metrics)
      : ER_CLIENT_INIT("fordyca.ros.support.d0.robot_arena_interactor"),
        m_nest_drop(metrics) {}

  robot_arena_interactor(robot_arena_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  robot_arena_interactor(const robot_arena_interactor&) = delete;
  robot_arena_interactor& operator=(const robot_arena_interactor&) = delete;

  /**
   * \brief The actual handling function for the interactions.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  fsupport::interactor_status operator()(TController& controller,
                                         const rtypes::timestep& t) {
    if (controller.is_carrying_block()) {
      return m_nest_drop(controller, t);
    } else { /* The robot has no block item */
      return m_free_pickup(controller, t);
    }
  }

 private:
  using pickup_spec = mpl::free_block_pickup_spec<
    controller::d0::reactive_typelist,
   fcrd0::events::free_block_pickup_visitor
    >;
  using drop_spec = mpl::nest_block_drop_spec<
    controller::d0::reactive_typelist,
    fcrd0::events::nest_block_drop_visitor
    >;

  /* clang-format off */
  free_block_pickup_interactor<TController, pickup_spec> m_free_pickup{};
  nest_block_drop_interactor<TController, drop_spec>     m_nest_drop;
  /* clang-format on */
};

NS_END(d0, support, ros, fordyca);

