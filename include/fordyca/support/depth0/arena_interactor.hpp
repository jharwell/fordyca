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
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <string>

#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "rcppsw/er/server.hpp"
#include "fordyca/metrics/fsm/goal_acquisition_metrics.hpp"
#include "fordyca/fsm/block_transporter.hpp"
#include "fordyca/support/depth0/stateless_metrics_aggregator.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

using acquisition_goal_type = metrics::fsm::goal_acquisition_metrics::goal_type;
using transport_goal_type = fsm::block_transporter::goal_type;

NS_START(depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/

/**
 * @class arena_interactor
 * @ingroup support
 *
 * @brief Handle's a robot's interactions with the environment on each timestep.
 *
 * Including:
 * - Picking up a free block.
 * - Dropping a carried block in the nest.
 */
template <typename T>
class arena_interactor : public rcppsw::er::client {
 public:
  arena_interactor(std::shared_ptr<rcppsw::er::server> server,
                   std::shared_ptr<representation::arena_map> map,
                   stateless_metrics_aggregator *metrics_agg,
                   argos::CFloorEntity* floor)
      : client(server),
        m_floor(floor),
        m_metrics_agg(metrics_agg),
        m_map(map) {}

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
    } else {
      handle_free_block_pickup(controller, timestep);
    }
  }

 protected:
  /**
   * @brief Determine if a robot is waiting to pick up a free block, and if it
   * is actually on a free block, send it the \ref free_block_pickup event.
   *
   * @return \c TRUE if the robot was sent the \ref free_block_pickup event,
   * \c FALSE otherwise.
   */
  bool handle_free_block_pickup(T& controller, uint timestep) {
    if (controller.goal_acquired() &&
        acquisition_goal_type::kBlock == controller.acquisition_goal()) {
      /* Check whether the foot-bot is actually on a block */
      int block = utils::robot_on_block(controller, *m_map);
      if (-1 != block) {
        events::free_block_pickup pickup_op(rcppsw::er::g_server,
                                            m_map->blocks()[block],
                                            utils::robot_id(controller),
                                            timestep);
        controller.visitor::template visitable_any<T>::accept(pickup_op);
        m_map->accept(pickup_op);

        /* The floor texture must be updated */
        m_floor->SetChanged();
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Determine if a robot is waiting to drop a block in the nest, and if
   * so send it the \ref nest_block_drop event.
   *
   * @return \c TRUE if the robot was sent the \ref nest_block_drop event, \c FALSE
   * otherwise.
   */
  bool handle_nest_block_drop(T& controller, uint timestep) {
    if (controller.in_nest() &&
        transport_goal_type::kNest == controller.block_transport_goal()) {
      /*
       * Gather block transport metrics before event processing and they get
       * reset.
       */
      controller.block()->nest_drop_time(timestep);
      m_metrics_agg->collect_from_block(controller.block().get());

      events::nest_block_drop drop_op(rcppsw::er::g_server,
                                      controller.block(),
                                      timestep);

      /* Update arena map state due to a block nest drop */
      m_map->accept(drop_op);

      /* Actually drop the block */
      controller.visitor::template visitable_any<T>::accept(drop_op);

      /* The floor texture must be updated */
      m_floor->SetChanged();
      return true;
    }
    return false;
  }

  std::shared_ptr<representation::arena_map>& map(void) { return m_map; }
  argos::CFloorEntity* floor(void) const { return m_floor; }

 private:
  // clang-format off
  argos::CFloorEntity*                       m_floor;
  stateless_metrics_aggregator*              m_metrics_agg;
  std::shared_ptr<representation::arena_map> m_map;
  // clang-format on
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_ARENA_INTERACTOR_HPP_ */
