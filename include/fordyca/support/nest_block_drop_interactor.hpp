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

#ifndef INCLUDE_FORDYCA_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/none.hpp>

#include "cosm/interactors/base_nest_block_process.hpp"

#include "fordyca/fsm/foraging_acq_goal.hpp"
#include "fordyca/fsm/foraging_transport_goal.hpp"
#include "fordyca/metrics/blocks/block_manip_events.hpp"
#include "fordyca/support/tv/block_op_src.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class nest_block_drop_interactor
 * \ingroup support
 *
 * \brief Handle's a robot's (possible) \ref nest_block_drop event on a given
 * timestep.
 */
template <typename TController, typename TControllerSpecMap>
class nest_block_drop_interactor final
    : public cinteractors::base_nest_block_process<TController,
                                                   TControllerSpecMap> {
 public:
  using typename cinteractors::
      base_nest_block_process<TController, TControllerSpecMap>::arena_map_type;
  using typename cinteractors::base_nest_block_process<
      TController,
      TControllerSpecMap>::penalty_handler_type;
  using typename cinteractors::
      base_nest_block_process<TController, TControllerSpecMap>::metrics_agg_type;

  nest_block_drop_interactor(arena_map_type* const map,
                             metrics_agg_type* const metrics_agg,
                             argos::CFloorEntity* const floor,
                             tv::env_dynamics* const envd)
      : cinteractors::base_nest_block_process<TController, TControllerSpecMap>(
            map,
            metrics_agg,
            floor,
            envd->penalty_handler(tv::block_op_src::ekNEST_DROP)) {}

  nest_block_drop_interactor(nest_block_drop_interactor&&) = default;

  /* Not copy-constructible/assignable by default. */
  nest_block_drop_interactor(const nest_block_drop_interactor&) = delete;
  nest_block_drop_interactor&
  operator=(const nest_block_drop_interactor&) = delete;

  void robot_penalty_init(const TController& controller,
                          const rtypes::timestep& t,
                          penalty_handler_type* handler) override {
    handler->penalty_init(
        controller, t, tv::block_op_src::ekNEST_DROP, boost::none);
  }

  bool robot_goal_acquired(const TController& controller) const override {
    return controller.goal_acquired() && fsm::foraging_transport_goal::ekNEST ==
                                             controller.block_transport_goal();
  }

  void robot_previsit_hook(TController& controller,
                           const ctv::temporal_penalty& penalty) const override {
    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     */
    controller.block_manip_recorder()->record(
        metrics::blocks::block_manip_events::ekFREE_DROP, penalty.penalty());
  }
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_ */
