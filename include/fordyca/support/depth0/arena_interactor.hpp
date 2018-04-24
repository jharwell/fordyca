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
#include "fordyca/metrics/block_transport_metrics_collector.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

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
  arena_interactor(const std::shared_ptr<rcppsw::er::server>& server,
                   const std::shared_ptr<representation::arena_map>& map,
                   argos::CFloorEntity* floor)
      : client(server),
        m_floor(floor),
        m_map(map) {}

  arena_interactor& operator=(const arena_interactor& other) = delete;
  arena_interactor(const arena_interactor& other) = delete;

  /**
   * @brief The actual handling function for the interactions.
   *
   * @param controller The controller to handle interactions for.
   * @param collector  The block metrics collector (block collection metrics are
   * somewhat different than others, so collection needs to be treated
   * specially).
   */
  void operator()(T& controller, metrics::block_transport_metrics_collector& collector) {
    if (controller.is_carrying_block()) {
      handle_nest_block_drop(controller, collector);
    } else {
      handle_free_block_pickup(controller);
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
  bool handle_free_block_pickup(T& controller) {
    if (controller.block_acquired()) {
      /* Check whether the foot-bot is actually on a block */
      int block = utils::robot_on_block(controller, *m_map);
      if (-1 != block) {
        events::free_block_pickup pickup_op(rcppsw::er::g_server,
                                            m_map->blocks()[block],
                                            utils::robot_id(controller));
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
  bool handle_nest_block_drop(T& controller,
      metrics::block_transport_metrics_collector& collector) {
    if (controller.in_nest() && controller.is_transporting_to_nest()) {
      /* Update arena map state due to a block nest drop */
      events::nest_block_drop drop_op(rcppsw::er::g_server, controller.block());

      /* update block transport metrics */
      collector.accept(drop_op);

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
  std::shared_ptr<representation::arena_map> m_map;
  // clang-format on
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_ARENA_INTERACTOR_HPP_ */
