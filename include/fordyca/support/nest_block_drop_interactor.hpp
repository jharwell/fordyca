/**
 * @file nest_block_drop_interactor.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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
#include <argos3/core/simulator/entity/floor_entity.h>
#include <string>

#include "fordyca/ds/arena_map.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/support/block_op_penalty_handler.hpp"
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

using transport_goal_type = fsm::block_transporter::goal_type;

namespace er = rcppsw::er;

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class nest_block_drop_interactor
 * @ingroup support
 *
 * @brief Handle's a robot's (possible) \ref nest_blop_drop event on a given
 * timestep.
 */
template <typename T>
class nest_block_drop_interactor
    : public er::client<nest_block_drop_interactor<T>> {
 public:
  nest_block_drop_interactor(ds::arena_map* const map,
                             depth0::depth0_metrics_aggregator* const metrics_agg,
                             argos::CFloorEntity* const floor,
                             const ct::waveform_params* const block_penalty)
      : ER_CLIENT_INIT("fordyca.support.depth0.nest_block_drop_interactor"),
        m_floor(floor),
        m_metrics_agg(metrics_agg),
        m_map(map),
        m_penalty_handler(map, block_penalty, "Nest block drop") {}
  nest_block_drop_interactor& operator=(
      const nest_block_drop_interactor& other) = delete;
  nest_block_drop_interactor(const nest_block_drop_interactor& other) = delete;

  /**
   * @brief The actual handling function for the robot-arena nest block drop
   * interaction.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    if (m_penalty_handler.is_serving_penalty(controller)) {
      if (m_penalty_handler.penalty_satisfied(controller, timestep)) {
        finish_nest_block_drop(controller, timestep);
      }
    } else {
      m_penalty_handler.penalty_init(controller,
                                     block_op_src::kSrcNestDrop,
                                     timestep);
    }
  }

  block_op_penalty_handler<T>* penalty_handler(void) {
    return &m_penalty_handler;
  }

 private:
  /**
   * @brief Determine if a robot is waiting to drop a block in the nest, and if
   * so send it the \ref nest_block_drop event.
   */
  void finish_nest_block_drop(T& controller, uint timestep) {
    ER_ASSERT(controller.in_nest(), "Controller not in nest");
    ER_ASSERT(transport_goal_type::kNest == controller.block_transport_goal(),
              "Controller still has nest as goal");
    ER_ASSERT(m_penalty_handler.is_serving_penalty(controller),
              "Controller not serving drop penalty");
    /*
     * More than 1 robot can drop a block in a timestep, so we have to
     * search for this robot's controller.
     */
    const temporal_penalty<T>& p = *m_penalty_handler.find(controller);

    perform_nest_block_drop(controller, p, timestep);
    m_penalty_handler.remove(p);
    ER_ASSERT(!m_penalty_handler.is_serving_penalty(controller),
              "Multiple instances of same controller serving drop penalty");
  }

  /**
   * @brief Perform the actual picking up of a free block once all
   * preconditions have been satisfied.
   */
  void perform_nest_block_drop(T& controller,
                               const temporal_penalty<T>& penalty,
                               uint timestep) {
    /*
     * We have to do this asynchronous to the rest of metric collection, because
     * the nest block drop event resets block metrics.
     */
    controller.block()->nest_drop_time(timestep);
    m_metrics_agg->collect_from_block(controller.block().get());

    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     */
    controller.penalty_served(penalty.penalty());

    events::nest_block_drop drop_op(controller.block(), timestep);

    /* Update arena map state due to a block nest drop */
    m_map->accept(drop_op);

    /* Actually drop the block */
    controller.visitor::template visitable_any<T>::accept(drop_op);

    /* The floor texture must be updated */
    m_floor->SetChanged();
  }

  // clang-format off
  argos::CFloorEntity*             const   m_floor;
  depth0::depth0_metrics_aggregator* const m_metrics_agg;
  ds::arena_map* const                     m_map;
  block_op_penalty_handler<T>              m_penalty_handler;
  // clang-format on
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_ */
