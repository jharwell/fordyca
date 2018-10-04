/**
 * @file arena_interactor.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_ARENA_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_ARENA_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <argos3/core/simulator/entity/floor_entity.h>

#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/events/block_vanished.hpp"
#include "fordyca/ds/arena_map.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/support/depth0/stateless_metrics_aggregator.hpp"
#include "fordyca/support/block_op_penalty_handler.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

NS_START(depth0);
namespace er = rcppsw::er;

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class arena_interactor
 * @ingroup support
 *
 * @brief Handle's a robot's interactions with the environment on each timestep:
 *
 * - Picking up a free block (possibly with penalty).
 * - Dropping a carried block in the nest (possibly with a penalty).
 */
template <typename T>
class arena_interactor : public er::client<arena_interactor<T>> {
 public:
  arena_interactor(ds::arena_map* const map,
                   stateless_metrics_aggregator *const metrics_agg,
                   argos::CFloorEntity* const floor,
                   const ct::waveform_params* const block_penalty)
      : ER_CLIENT_INIT("fordyca.support.depth0.arena_interactor"),
        m_floor(floor),
        m_metrics_agg(metrics_agg),
        m_map(map),
        m_free_pickup_handler(map, block_penalty),
        m_nest_drop_handler(map, block_penalty) {}
  arena_interactor& operator=(const arena_interactor& other) = delete;
  arena_interactor(const arena_interactor& other) = delete;

  /**
   * @brief The actual handling function for the interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param timestep The current timestep.
   */
  void operator()(T& controller, uint timestep) {
    if (controller.is_carrying_block()) {
      handle_nest_block_drop(controller, timestep);
    } else { /* The foot-bot has no block item */
      handle_free_block_pickup(controller, timestep);
    }
  }

 protected:
  typedef typename block_op_penalty_handler<T>::penalty_src penalty_type;
  void handle_free_block_pickup(T& controller, uint timestep) {
    if (m_free_pickup_handler.is_serving_penalty(controller)) {
      if (m_free_pickup_handler.penalty_satisfied(controller,
                                                  timestep)) {
        finish_free_block_pickup(controller, timestep);
      }
    } else {
      m_free_pickup_handler.penalty_init(controller,
                                         penalty_type::kFreePickup,
                                         timestep);
    }
  }

  void handle_nest_block_drop(T& controller, uint timestep) {
    if (m_nest_drop_handler.is_serving_penalty(controller)) {
      if (m_nest_drop_handler.penalty_satisfied(controller,
                                                timestep)) {
        finish_nest_block_drop(controller, timestep);
      }
    } else {
      m_nest_drop_handler.penalty_init(controller,
                                       penalty_type::kNestDrop,
                                       timestep);
    }
  }

  /**
   * @brief Determine if a robot is waiting to pick up a free block, and if it
   * is actually on a free block, send it the \ref free_block_pickup event.
   */
  void finish_free_block_pickup(T& controller, uint timestep) {
    ER_ASSERT(controller.goal_acquired() &&
              acquisition_goal_type::kBlock == controller.acquisition_goal(),
              "Controller not waiting for free block pickup");
    ER_ASSERT(m_free_pickup_handler.is_serving_penalty(controller),
              "Controller not serving pickup penalty");

    /*
     * More than 1 robot can pick up a block in a timestep, so we have to
     * search for this robot's controller
     */
    const temporal_penalty<T>& p = *m_free_pickup_handler.find(controller);

    /*
     * If two robots both are serving penalties on the same ramp block (possible
     * because a ramp block spans 2 squares), then whichever robot finishes
     * first will correctly take the block via \ref free_block_pickup, and the
     * second one will attempt to perform the pickup on a block that is already
     * out of sight, resulting in a boost index out of bounds assertion. See
     * #410.
     */
    if (-1 == utils::robot_on_block(controller, *map())) {
      ER_WARN("%s cannot pickup block%d: No such block",
              controller.GetId().c_str(),
              m_free_pickup_handler.find(controller)->id());
      events::block_vanished vanished(p.id());
      controller.visitor::template visitable_any<T>::accept(vanished);
    } else {
      perform_free_block_pickup(controller, p, timestep);
      floor()->SetChanged();
    }
    m_free_pickup_handler.remove(p);
    ER_ASSERT(!m_free_pickup_handler.is_serving_penalty(controller),
              "Multiple instances of same controller serving block pickup penalty");
  }

  /**
   * @brief Perform the actual picking up of a free block once all
   * preconditions have been satisfied.
   */
  void perform_free_block_pickup(T& controller,
                                 const temporal_penalty<T>& penalty,
                                 uint timestep) {
    auto it = std::find_if(m_map->blocks().begin(),
                           m_map->blocks().end(),
                           [&](const auto& b) {
                             return b->id() == penalty.id();
                           });
    ER_ASSERT(it != m_map->blocks().end(),
              "Block%d from penalty does not exist",
              penalty.id());
    ER_ASSERT((*it)->real_loc() != representation::base_block::kOutOfSightRLoc,
              "Attempt to pick up out of sight block%d",
              (*it)->id());
    events::free_block_pickup pickup_op(*it,
                                        utils::robot_id(controller),
                                        timestep);

    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     */
    controller.penalty_served(penalty.penalty());
    controller.visitor::template visitable_any<T>::accept(pickup_op);

    m_map->accept(pickup_op);

    /* The floor texture must be updated */
    m_floor->SetChanged();
  }

  /**
   * @brief Determine if a robot is waiting to drop a block in the nest, and if
   * so send it the \ref nest_block_drop event.
   */
  void finish_nest_block_drop(T& controller, uint timestep) {
    ER_ASSERT(controller.in_nest(),
              "Controller not in nest");
    ER_ASSERT(transport_goal_type::kNest == controller.block_transport_goal(),
              "Controller still has nest as goal");
    ER_ASSERT(m_nest_drop_handler.is_serving_penalty(controller),
              "Controller not serving drop penalty");
    /*
     * More than 1 robot can drop a block in a timestep, so we have to
     * search for this robot's controller.
     */
    const temporal_penalty<T>& p = *m_nest_drop_handler.find(controller);

    perform_nest_block_drop(controller, p, timestep);
    m_nest_drop_handler.remove(p);
    ER_ASSERT(!m_nest_drop_handler.is_serving_penalty(controller),
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

  ds::arena_map* map(void) { return m_map; }
  const ds::arena_map* map(void) const { return m_map; }
  const argos::CFloorEntity* floor(void) const { return m_floor; }
  argos::CFloorEntity* floor(void) { return m_floor; }
  block_op_penalty_handler<T>& free_pickup_penalty_handler(void) {
    return m_free_pickup_handler;
  }
  block_op_penalty_handler<T>& nest_drop_penalty_handler(void) {
    return m_nest_drop_handler;
  }
  const block_op_penalty_handler<T>& free_pickup_penalty_handler(void) const {
    return m_free_pickup_handler;
  }
  const block_op_penalty_handler<T>& nest_drop_penalty_handler(void) const {
    return m_nest_drop_handler;
  }

 private:
  // clang-format off
  argos::CFloorEntity*             const m_floor;
  stateless_metrics_aggregator*    const m_metrics_agg;
  ds::arena_map* const                   m_map;
  block_op_penalty_handler<T>            m_free_pickup_handler;
  block_op_penalty_handler<T>            m_nest_drop_handler;
  // clang-format on
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_ARENA_INTERACTOR_HPP_ */
