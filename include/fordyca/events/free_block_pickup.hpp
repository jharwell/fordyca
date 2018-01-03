/**
 * @file free_block_pickup.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "fordyca/events/cell_op.hpp"
#include "fordyca/events/block_pickup_event.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;

namespace fsm {
namespace depth0 { class stateless_foraging_fsm; class stateful_foraging_fsm; }
namespace depth1 { class block_to_cache_fsm; }
}
namespace controller {
namespace depth0 {
class stateless_foraging_controller;
class stateful_foraging_controller;
}}

namespace tasks { class generalist; class forager; }

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class free_block_pickup
 * @ingroup events
 *
 * @brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache).
 */
class free_block_pickup : public cell_op,
                          public rcppsw::er::client,
                          public block_pickup_event,
                          public visitor::visit_set<controller::depth0::stateless_foraging_controller,
                                                    controller::depth0::stateful_foraging_controller,
                                                    fsm::depth0::stateless_foraging_fsm,
                                                    fsm::depth0::stateful_foraging_fsm,
                                                    fsm::depth1::block_to_cache_fsm,
                                                    tasks::generalist,
                                                    tasks::forager> {
 public:
  free_block_pickup(const std::shared_ptr<rcppsw::er::server>& server,
                    representation::block* block, size_t robot_index);
  ~free_block_pickup(void) { client::rmmod(); }

  free_block_pickup(const free_block_pickup& op) = delete;
  free_block_pickup& operator=(const free_block_pickup& op) = delete;

  /* stateless foraging */
  void visit(representation::arena_map& map) override;
  void visit(representation::cell2D& cell) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(representation::block& block) override;
  void visit(controller::depth0::stateless_foraging_controller& controller) override;
  void visit(fsm::depth0::stateless_foraging_fsm& fsm) override;

  /* stateful foraging */
  void visit(representation::perceived_arena_map& map) override;
  void visit(representation::perceived_cell2D& cell) override;
  void visit(fsm::depth0::stateful_foraging_fsm& fsm) override;
  void visit(controller::depth0::stateful_foraging_controller& controller) override;

  /* depth1 foraging */
  void visit(controller::depth1::foraging_controller& controller) override;
  void visit(fsm::depth1::block_to_cache_fsm& fsm) override;
  void visit(fsm::block_to_nest_fsm& fsm) override;
  void visit(tasks::generalist& task) override;
  void visit(tasks::forager& task) override;

 private:
  size_t m_robot_index;
  representation::block* m_block;
  std::shared_ptr<rcppsw::er::server> m_server;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_PICKUP_HPP_ */
