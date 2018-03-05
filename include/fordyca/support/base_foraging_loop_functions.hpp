/**
 * @file base_foraging_loop_functions.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_BASE_FORAGING_LOOP_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_BASE_FORAGING_LOOP_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/simulator/loop_functions.h>
#include <string>

#include "fordyca/events/free_block_pickup.hpp"
#include "fordyca/events/nest_block_drop.hpp"
#include "fordyca/metrics/block_metrics_collector.hpp"
#include "fordyca/representation/arena_map.hpp"
#include "fordyca/representation/line_of_sight.hpp"
#include "fordyca/support/loop_functions_utils.hpp"
#include "rcppsw/er/server.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class base_foraging_loop_functions
 * @ingroup support
 *
 * @brief The base loop functions in FORDYCA that all other loop functions
 * inherit from.
 *
 * This class is not a functional set of loop functions, but it provides
 * functions needed across multiple derived classes, but functionality that
 * could not just be free functions because they require access to members in
 * the \ref argos::CLoopFunctions class.
 */
class base_foraging_loop_functions : public argos::CLoopFunctions {
 public:
  base_foraging_loop_functions(void) = default;

  base_foraging_loop_functions(const base_foraging_loop_functions& s) = delete;
  base_foraging_loop_functions& operator=(
      const base_foraging_loop_functions& s) = delete;

  void Init(argos::TConfigurationNode&) override {
    m_floor = &GetSpace().GetFloorEntity();
  }

  /**
   * @brief Determine if a robot is waiting to pick up a free block, and if it
   * is actually on a free block, send it the \ref free_block_pickup event.
   *
   * @return \c TRUE if the robot was sent the \ref free_block_pickup event,
   * \c FALSE otherwise.
   */
  template <typename T>
  bool handle_free_block_pickup(argos::CFootBotEntity& robot,
                                representation::arena_map& map) {
    auto& controller =
        static_cast<T&>(robot.GetControllableEntity().GetController());

    if (controller.block_acquired()) {
      /* Check whether the foot-bot is actually on a block */
      int block = utils::robot_on_block(robot, map);
      if (-1 != block) {
        events::free_block_pickup pickup_op(rcppsw::er::g_server,
                                            &map.blocks()[block],
                                            utils::robot_id(robot));
        controller.visitor::template visitable_any<T>::accept(pickup_op);
        map.accept(pickup_op);

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
  template <typename T>
  bool handle_nest_block_drop(argos::CFootBotEntity& robot,
                              representation::arena_map& map,
                              metrics::block_metrics_collector& block_collector) {
    auto& controller =
        static_cast<T&>(robot.GetControllableEntity().GetController());
    if (controller.in_nest() && controller.is_transporting_to_nest()) {
      /* Update arena map state due to a block nest drop */
      events::nest_block_drop drop_op(rcppsw::er::g_server, controller.block());

      /* update block carries */
      block_collector.accept(drop_op);

      map.accept(drop_op);

      /* Actually drop the block */
      controller.visitor::template visitable_any<T>::accept(drop_op);

      /* The floor texture must be updated */
      m_floor->SetChanged();
      return true;
    }
    return false;
  }

 protected:
  argos::CFloorEntity* floor(void) const { return m_floor; }

 private:
  argos::CFloorEntity* m_floor{nullptr};
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_BASE_FORAGING_LOOP_FUNCTIONS_HPP_ */
