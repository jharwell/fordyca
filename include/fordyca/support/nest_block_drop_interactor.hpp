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
#include <string>

#include <argos3/core/simulator/entity/floor_entity.h>

#include "fordyca/ds/arena_map.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/support/depth0/depth0_metrics_aggregator.hpp"
#include "fordyca/support/interactor_status.hpp"
#include "fordyca/support/tv/tv_manager.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * \class nest_block_drop_interactor
 * \ingroup fordyca support
 *
 * \brief Handle's a robot's (possible) \ref nest_block_drop event on a given
 * timestep.
 */
template <typename T>
class nest_block_drop_interactor
    : public rer::client<nest_block_drop_interactor<T>> {
 public:
  nest_block_drop_interactor(ds::arena_map* const map,
                             depth0::depth0_metrics_aggregator* const metrics_agg,
                             argos::CFloorEntity* const floor,
                             tv::tv_manager* tv_manager)
      : ER_CLIENT_INIT("fordyca.support.nest_block_drop_interactor"),
        m_floor(floor),
        m_metrics_agg(metrics_agg),
        m_map(map),
        m_penalty_handler(
            tv_manager->penalty_handler<T>(tv::block_op_src::ekNEST_DROP)) {}

  /**
   * \brief Interactors should generally NOT be copy constructable/assignable,
   * but is needed to use these classes with boost::variant.
   *
   * \todo Supposedly in recent versions of boost you can use variants with
   * move-constructible-only types (which is what this class SHOULD be), but I
   * cannot get this to work (the default move constructor needs to be noexcept
   * I think, and is not being interpreted as such).
   */
  nest_block_drop_interactor(const nest_block_drop_interactor& other) = default;
  nest_block_drop_interactor& operator=(
      const nest_block_drop_interactor& other) = delete;

  /**
   * \brief The actual handling function for the robot-arena nest block drop
   * interaction.
   *
   * \param controller The controller to handle interactions for.
   * \param t The current timestep.
   */
  interactor_status operator()(T& controller, const rtypes::timestep& t) {
    if (m_penalty_handler->is_serving_penalty(controller)) {
      if (m_penalty_handler->is_penalty_satisfied(controller, t)) {
        finish_nest_block_drop(controller, t);
        return interactor_status::ekNEST_BLOCK_DROP;
      }
    } else {
      m_penalty_handler->penalty_init(controller,
                                      tv::block_op_src::ekNEST_DROP,
                                      t);
    }
    return interactor_status::ekNO_EVENT;
  }

 private:
  /**
   * \brief Determine if a robot is waiting to drop a block in the nest, and if
   * so send it the \ref nest_block_drop event.
   */
  void finish_nest_block_drop(T& controller, rtypes::timestep t) {
    ER_ASSERT(controller.in_nest(), "Controller not in nest");
    ER_ASSERT(fsm::foraging_transport_goal::type::ekNEST ==
                  controller.block_transport_goal(),
              "Controller still has nest as goal");
    ER_ASSERT(m_penalty_handler->is_serving_penalty(controller),
              "Controller not serving drop penalty");
    /*
     * More than 1 robot can drop a block in a timestep, so we have to
     * search for this robot's controller.
     */
    const auto& p = *m_penalty_handler->penalty_find(controller);
    perform_nest_block_drop(controller, p, t);
    m_penalty_handler->penalty_remove(p);
    ER_ASSERT(!m_penalty_handler->is_serving_penalty(controller),
              "Multiple instances of same controller serving drop penalty");
  }

  /**
   * \brief Perform the actual picking up of a free block once all
   * preconditions have been satisfied.
   */
  void perform_nest_block_drop(T& controller,
                               const tv::temporal_penalty<T>& penalty,
                               rtypes::timestep t) {
    /*
     * We have to do this asynchronous to the rest of metric collection, because
     * the \ref nest_block_drop event resets block metrics.
     */
    controller.block()->dest_drop_time(t);
    m_metrics_agg->collect_from_block(controller.block());

    /*
     * Penalty served needs to be set here rather than in the free block pickup
     * event, because the penalty is generic, and the event handles concrete
     * classes--no clean way to mix the two.
     */
    controller.block_manip_collator()->penalty_served(penalty.penalty());

    events::nest_block_drop_visitor drop_op(controller.block_release(), t);

    /*
     * Order of visitation must be:
     *
     * 1. Arena map
     * 2. Controller
     *
     * In order for \ref events::nest_block_drop to process properly.
     */
    /* Update arena map state due to a block nest drop */
    drop_op.visit(*m_map);

    /* Actually drop the block */
    drop_op.visit(controller);

    /* The floor texture must be updated */
    m_floor->SetChanged();
  }

  /* clang-format off */
  argos::CFloorEntity* const               m_floor;
  depth0::depth0_metrics_aggregator* const m_metrics_agg;
  ds::arena_map* const                     m_map;
  tv::block_op_penalty_handler<T>* const   m_penalty_handler;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_NEST_BLOCK_DROP_INTERACTOR_HPP_ */
