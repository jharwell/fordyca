/**
 * @file cached_block_pickup.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/concrete_arena_op.hpp"
#include "rcppsw/common/er_client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace representation { class perceived_arena_map; }
namespace representation { class cache; class block; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cached_block_pickup : public concrete_arena_op,
                            public rcppsw::common::er_client,
                            public visitor::can_visit<representation::perceived_arena_map>,
                            public visitor::can_visit<representation::cache> {
 public:
  cached_block_pickup(const std::shared_ptr<rcppsw::common::er_server>& server,
                      representation::cache* cache, size_t robot_index);
  ~cached_block_pickup(void) { er_client::rmmod(); }

  /**
   * @brief Update the arena_map with the block pickup event by making the block
   * seem to disappear by moving it out of sight.
   *
   * @param map The arena_map.
   */
  void visit(representation::arena_map& map) override;

  /**
   * @brief Handle the event of a robot picking up a block, making updates to
   * the arena map as necessary.
   *
   * @param map The robot's arena map.
   */
  void visit(representation::perceived_arena_map& map) override;

  /**
   * @brief Update a block with the knowledge that it is now carried by a robot.
   */
  void visit(representation::block& block) override;

  void visit(fsm::memory_foraging_fsm& fsm) override;

  /**
   * @brief Pickup a block the robot is currently on top of, updating state as appropriate.
   *
   * This needs to be here, rather than in the FSM, because picking up blocks
   * needs to be handled in the loop functions so the area can correctly be drawn
   * each timestep.
   */
  void visit(controller::memory_foraging_controller& controller) override;

 private:
  cached_block_pickup(const cached_block_pickup& op) = delete;
  cached_block_pickup& operator=(const cached_block_pickup& op) = delete;

  size_t m_robot_index;
  representation::cache* m_cache;
  representation::block* m_block;
  std::shared_ptr<rcppsw::common::er_server> m_server;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHED_BLOCK_PICKUP_HPP_ */
