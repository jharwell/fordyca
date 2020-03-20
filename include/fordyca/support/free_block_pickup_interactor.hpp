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

#ifndef INCLUDE_FORDYCA_SUPPORT_FREE_BLOCK_PICKUP_INTERACTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_FREE_BLOCK_PICKUP_INTERACTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include <argos3/core/simulator/entity/floor_entity.h>

#include "cosm/arena/arena_map.hpp"
#include "cosm/arena/operations/free_block_pickup.hpp"

#include "fordyca/events/block_vanished.hpp"
#include "fordyca/events/robot_free_block_pickup.hpp"
#include "fordyca/fsm/foraging_goal_type.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/support/tv/env_dynamics.hpp"
#include "fordyca/support/utils/loop_utils.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class free_block_pickup_interactor
 * \ingroup support
 *
 * \brief Handle's a robot's (possible) \ref free_block_pickup event on a given
 * timestep.
 */
template <typename T>
class free_block_pickup_interactor
    : public rer::client<free_block_pickup_interactor<T>> {
 public:
  free_block_pickup_interactor(carena::arena_map* const map,
                               argos::CFloorEntity* const floor,
                               tv::env_dynamics* envd)
      : ER_CLIENT_INIT("fordyca.support.free_block_pickup_interactor"),
        m_floor(floor),
        m_map(map),
        m_penalty_handler(
            envd->penalty_handler(tv::block_op_src::ekFREE_PICKUP)) {}

  /**
   * \brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * \todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  free_block_pickup_interactor(const free_block_pickup_interactor& other) =
      default;
  free_block_pickup_interactor& operator=(const free_block_pickup_interactor&) =
      delete;

  /**
   * \brief The actual handlipng function for the free block pickup arena-robot
   * interaction.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status operator()(T& controller, const rtypes::timestep& t) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        finish_free_block_pickup(controller, t);
        return interactor_status::ekFREE_BLOCK_PICKUP;
      }
    } else {
      m_penalty_handler->penalty_init(controller,
                                      tv::block_op_src::ekFREE_PICKUP,
                                      t);
    }
    return interactor_status::ekNO_EVENT;
  }

 private:
  /**
   * \brief Determine if a robot is waiting to pick up a free block, and if it
   * is actually on a free block, send it the \ref free_block_pickup event.
   */
  void finish_free_block_pickup(T& controller, rtypes::timestep t) {
    ER_ASSERT(controller.goal_acquired() &&
                  fsm::foraging_acq_goal::type::ekBLOCK ==
                      controller.acquisition_goal(),
              "Controller not waiting for free block pickup");
    ER_ASSERT(m_penalty_handler->is_serving_penalty(controller),
              "Controller not serving pickup penalty");
    ER_ASSERT(!controller.is_carrying_block(),
              "Controller is already carrying block%d",
              controller.block()->id().v());
    /*
     * More than 1 robot can pick up a block in a timestep, so we have to
     * search for this robot's controller.
     */
    const auto& p = *m_penalty_handler->penalty_find(controller);

    /*
     * We cannot just lock around the critical arena map updates
     * here in order to make this section thread safe because if two threads
     * updating two robots both having finished serving their penalty this
     * timestep manage to pass the check to actually perform the block pickup
     * before one of them actually finishes picking up a block, then the second
     * one will not get the necessary \ref block_vanished event. See #594.
     */
    m_map->block_mtx()->lock();

    /*
     * If two robots both are serving penalties on the same ramp block (possible
     * because a ramp block spans 2 squares), then whichever robot finishes
     * first will correctly take the block via \ref free_block_pickup, and the
     * second one will attempt to perform the pickup on a block that is already
     * out of sight, resulting in a boost index out of bounds assertion. See
     * #410.
     *
     * Furthermore, it is ALSO possible that while the second robot is still
     * waiting to serve its penalty, and the first robot has already picked up
     * the ramp block, that the arena distributes a new block onto the square
     * that the robot is currently occupying. Thus, when it has served its
     * penalty, we need to check that the ID of the block the robot is on
     * matches the ID of the block we originally served the penalty for (not
     * just checking if it is not -1).
     */
    if (p.id() != utils::robot_on_block(controller, *m_map)) {
      ER_WARN("%s cannot pickup block%d: No such block",
              controller.GetId().c_str(),
              m_penalty_handler->penalty_find(controller)->id().v());
      events::block_vanished_visitor vanished_op(p.id());
      vanished_op.visit(controller);
    } else {
      perform_free_block_pickup(controller, p, t);
    }
    m_map->block_mtx()->unlock();

    m_penalty_handler->penalty_remove(p);
    ER_ASSERT(
        !m_penalty_handler->is_serving_penalty(controller),
        "Multiple instances of same controller serving block pickup penalty");
  }

  /**
   * \brief Perform the actual picking up of a free block once all
   * preconditions have been satisfied.
   */
  void perform_free_block_pickup(T& controller,
                                 const tv::temporal_penalty& penalty,
                                 const rtypes::timestep& t) {
    /* Holding block mutex not necessary here, but does not hurt */
    auto it =
        std::find_if(m_map->blocks().begin(),
                     m_map->blocks().end(),
                     [&](const auto& b) { return b->id() == penalty.id(); });
    ER_ASSERT(it != m_map->blocks().end(),
              "Block%d from penalty does not exist?",
              penalty.id().v());
    ER_ASSERT(!(*it)->is_out_of_sight(),
              "Attempt to pick up out of sight block%d",
              (*it)->id().v());
    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     */
    controller.block_manip_collator()->penalty_served(penalty.penalty());
    events::robot_free_block_pickup_visitor rpickup_op(*it,
                                                       controller.entity_id(),
                                                       t);
    caops::free_block_pickup_visitor apickup_op(*it,
                                                controller.entity_id(),
                                                t);

    /*
     * Visitation order must be:
     *
     * 1. Arena map
     * 2. Controller
     *
     * In order for the pickup event to process properly.
     */
    apickup_op.visit(*m_map);
    rpickup_op.visit(controller);

    /* The floor texture must be updated */
    m_floor->SetChanged();
  }

  /* clang-format off */
  argos::CFloorEntity*const           m_floor;
  carena::arena_map* const            m_map;
  tv::block_op_penalty_handler* const m_penalty_handler;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_FREE_BLOCK_PICKUP_HANDLER_HPP_ */
