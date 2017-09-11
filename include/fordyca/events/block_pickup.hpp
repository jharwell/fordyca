/**
 * @file block_pickup.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_op.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class random_foraging_controller;
class unpartitioned_task_controller;
} /* namespace controller */

namespace representation {
class arena_map;
class block;
} /* namespace representation */

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class block_pickup : public block_op, public rcppsw::common::er_client {
 public:
  block_pickup(const std::shared_ptr<rcppsw::common::er_server>& server,
               representation::block* block, size_t robot_index);
  ~block_pickup(void) { rmmod(); }

  /**
   * @brief Update the arena_map with the block pickup event by making the block
   * seem to disappear by moving it out of sight.
   *
   * @param map The arena_map.
   */
  void visit(representation::arena_map& map);

  /**
   * @brief Handle the event of a robot picking up a block, making updates to
   * the arena map as necessary.
   *
   * @param map The robot's arena map.
   */
  void visit(representation::perceived_arena_map& map);

  /**
   * @brief Update a block with the knowledge that it is now carried by a robot.
   */
  void visit(representation::block& block);

  /**
   * @brief Pickup a block the robot is currently on top of, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because picking up blocks
   * needs to be handled in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  void visit(controller::random_foraging_controller& controller);

  /**
   * @brief Pickup a block the robot is currently on top of, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because picking up blocks
   * needs to be handled in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  void visit(controller::unpartitioned_task_controller& controller);

 private:
  block_pickup(const block_pickup& op) = delete;
  block_pickup& operator=(const block_pickup& op) = delete;

  size_t m_robot_index;
  representation::block* m_block;
  std::shared_ptr<rcppsw::common::er_server> m_server;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_PICKUP_HPP_ */
