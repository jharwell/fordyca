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
#include "fordyca/events/block_pickup_event.hpp"
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;

namespace fsm {
namespace depth0 {
class stateless_fsm;
class stateful_fsm;
class free_block_to_nest_fsm;
} // namespace depth0
class block_to_goal_fsm;
} // namespace fsm
namespace controller {
namespace depth0 {
class stateless_controller;
class stateful_controller;
} // namespace depth0
namespace depth1 {
class greedy_partitioning_controller;
}
namespace depth2 {
class greedy_recpart_controller;
}
} // namespace controller

namespace tasks {
namespace depth0 {
class generalist;
}
namespace depth1 {
class harvester;
}
namespace depth2 {
class cache_starter;
class cache_finisher;
} // namespace depth2
} // namespace tasks

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
class free_block_pickup
    : public cell_op,
      public rcppsw::er::client<free_block_pickup>,
      public block_pickup_event,
      public visitor::visit_set<controller::depth0::stateless_controller,
                                controller::depth0::stateful_controller,
                                controller::depth1::greedy_partitioning_controller,
                                controller::depth2::greedy_recpart_controller,
                                fsm::depth0::stateless_fsm,
                                fsm::depth0::stateful_fsm,
                                fsm::depth0::free_block_to_nest_fsm,
                                fsm::block_to_goal_fsm,
                                tasks::depth0::generalist,
                                tasks::depth1::harvester,
                                tasks::depth2::cache_starter,
                                tasks::depth2::cache_finisher> {
 public:
  free_block_pickup(std::shared_ptr<representation::base_block> block,
                    uint robot_index,
                    uint timestep);
  ~free_block_pickup(void) override = default;

  free_block_pickup(const free_block_pickup& op) = delete;
  free_block_pickup& operator=(const free_block_pickup& op) = delete;

  /* stateless foraging */
  void visit(ds::arena_map& map) override;
  void visit(ds::cell2D& cell) override;
  void visit(fsm::cell2D_fsm& fsm) override;
  void visit(representation::base_block& block) override;
  void visit(controller::depth0::stateless_controller& controller) override;
  void visit(fsm::depth0::stateless_fsm& fsm) override;

  /* stateful foraging */
  void visit(ds::perceived_arena_map& map) override;
  void visit(fsm::depth0::stateful_fsm& fsm) override;
  void visit(controller::depth0::stateful_controller& controller) override;

  /* depth1 foraging */
  void visit(fsm::depth0::free_block_to_nest_fsm& fsm) override;
  void visit(
      controller::depth1::greedy_partitioning_controller& controller) override;
  void visit(fsm::block_to_goal_fsm& fsm) override;
  void visit(tasks::depth0::generalist& task) override;
  void visit(tasks::depth1::harvester& task) override;

  /* depth2 foraging */
  void visit(controller::depth2::greedy_recpart_controller& controller) override;
  void visit(tasks::depth2::cache_starter& task) override;
  void visit(tasks::depth2::cache_finisher& task) override;

 private:
  // clang-format off
  uint                                        m_timestep;
  uint                                        m_robot_index;
  std::shared_ptr<representation::base_block> m_block;
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_PICKUP_HPP_ */
