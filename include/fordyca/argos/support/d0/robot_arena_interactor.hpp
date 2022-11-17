/**
 * \file d0/robot_arena_interactor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/free_block_pickup_interactor.hpp"
#include "fordyca/argos/support/nest_block_drop_interactor.hpp"
#include "fordyca/argos/support/mpl/free_block_pickup_spec.hpp"
#include "fordyca/argos/support/mpl/nest_block_drop_spec.hpp"
#include "fordyca/argos/support/tv/env_dynamics.hpp"
#include "fordyca/controller/cognitive/d0/events/nest_block_drop.hpp"
#include "fordyca/controller/cognitive/d0/events/free_block_pickup.hpp"
#include "fordyca/controller/cognitive/d0/events/block_vanished.hpp"
#include "fordyca/controller/reactive/d0/events/nest_block_drop.hpp"
#include "fordyca/controller/reactive/d0/events/free_block_pickup.hpp"
#include "fordyca/controller/reactive/d0/events/block_vanished.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d0);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class robot_arena_interactor
 * \ingroup argos support d0
 *
 * \brief Handle's a robot's interactions with the environment on each timestep.
 *
 * Including:
 *
 * - Picking up a free block.
 * - Dropping a carried block in the nest.
 */
template <typename TController,
          typename TArenaMap>
class robot_arena_interactor final : public rer::client<
  robot_arena_interactor<TController,
                         TArenaMap>
  > {
 public:
  robot_arena_interactor(TArenaMap* const map,
                         fametrics::d0::d0_metrics_manager *const metrics_agg,
                         ::argos::CFloorEntity* const floor,
                         tv::env_dynamics* const envd)
      : ER_CLIENT_INIT("fordyca.argos.support.d0.robot_arena_interactor"),
        m_free_pickup(map,
                      floor,
                      envd->penalty_handler(tv::block_op_src::ekFREE_PICKUP)),
        m_nest_drop(map, metrics_agg, floor, envd) {}

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
    } else { /* The foot-bot has no block item */
      return m_free_pickup(controller, t);
    }
  }

 private:
  using robot_block_vanished_visitor_type = typename std::conditional<
   fcontroller::d0::is_cognitive<TController>::value,
   fccd0::events::block_vanished_visitor,
   fcrd0::events::block_vanished_visitor
   >::type;

  using robot_free_block_pickup_visitor_type = typename std::conditional<
   fcontroller::d0::is_cognitive<TController>::value,
   fccd0::events::free_block_pickup_visitor,
   fcrd0::events::free_block_pickup_visitor
    >::type;

  using robot_nest_block_drop_visitor_type = typename std::conditional<
   fcontroller::d0::is_cognitive<TController>::value,
   fccd0::events::nest_block_drop_visitor,
   fcrd0::events::nest_block_drop_visitor
    >::type;

  using pickup_spec = mpl::free_block_pickup_spec<
    controller::d0::typelist,
    robot_block_vanished_visitor_type,
    robot_free_block_pickup_visitor_type
    >;
  using drop_spec = mpl::nest_block_drop_spec<
    controller::d0::typelist,
    robot_nest_block_drop_visitor_type
    >;

  /* clang-format off */
  free_block_pickup_interactor<TController, pickup_spec> m_free_pickup;
  nest_block_drop_interactor<TController, drop_spec>     m_nest_drop;
  /* clang-format on */
};

NS_END(d0, support, argos, fordyca);

