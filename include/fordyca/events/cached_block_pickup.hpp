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
#include "rcppsw/common/er_client.hpp"
#include "fordyca/events/cell_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace representation {
class perceived_arena_map;
class cache;
class block;
class arena_map;
}
namespace fsm { class block_to_nest_fsm; }
namespace controller { namespace depth1 {class foraging_controller; }}
namespace tasks { class collector; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class cached_block_pickup : public cell_op,
                            public rcppsw::common::er_client,
                            public visitor::visit_set<controller::depth1::foraging_controller,
                                                      fsm::block_to_nest_fsm,
                                                      tasks::collector,
                                                      representation::block,
                                                      representation::arena_map,
                                                      representation::perceived_arena_map> {
 public:
  cached_block_pickup(const std::shared_ptr<rcppsw::common::er_server>& server,
                      representation::cache* cache, size_t robot_index);
  ~cached_block_pickup(void) { er_client::rmmod(); }

  /* depth1 foraging */
  /**
   * @brief Update the arena_map with the block pickup event by making the block
   * seem to disappear by moving it out of sight.
   *
   * @param map The arena_map.
   */
  void visit(representation::arena_map& map) override;

  void visit(representation::cell2D& cell) override;
  void visit(representation::cell2D_fsm& fsm) override;
  void visit(representation::perceived_cell2D& cell) override;

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

  void visit(fsm::block_to_nest_fsm& fsm) override;

  void visit(controller::depth1::foraging_controller& controller) override;
  void visit(tasks::collector& task) override;

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
