/**
 * @file block_vanished.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_VANISHED_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_VANISHED_HPP_

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

namespace fsm {
namespace depth0 {
class stateless_fsm;
class stateful_fsm;
} // namespace depth0
namespace depth1 {
class block_to_goal_fsm;
}
} // namespace fsm
namespace tasks {
namespace depth0 {
class generalist;
}
namespace depth1 {
class harvester;
} // namespace depth1
namespace depth2 {
class cache_starter;
class cache_finisher;
} // namespace depth2
} // namespace tasks

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * @class block_vanished
 * @ingroup events
 *
 * @brief Created whenever a robot is serving a block pickup penalty, but while
 * serving the penalty the block it is waiting for vanishes due to another
 * robot picking it up (ramp blocks only).
 */
class block_vanished
    : public rcppsw::er::client<block_vanished>,
      public visitor::visit_set<controller::depth0::stateless_controller,
                                controller::depth0::stateful_controller,
                                controller::depth1::greedy_partitioning_controller,
                                controller::depth2::greedy_recpart_controller,
                                tasks::depth0::generalist,
                                tasks::depth1::harvester,
                                tasks::depth2::cache_starter,
                                tasks::depth2::cache_finisher,
                                fsm::depth0::stateless_fsm,
                                fsm::depth0::stateful_fsm,
                                fsm::depth1::block_to_goal_fsm> {
 public:
  explicit block_vanished(uint block_id);
  ~block_vanished(void) override = default;

  block_vanished(const block_vanished& op) = delete;
  block_vanished& operator=(const block_vanished& op) = delete;

  /* depth0 foraging */
  void visit(controller::depth0::stateless_controller& controller) override;
  void visit(controller::depth0::stateful_controller& controller) override;
  void visit(tasks::depth0::generalist& task) override;
  void visit(fsm::depth0::stateless_fsm& fsm) override;
  void visit(fsm::depth0::stateful_fsm& fsm) override;

  /* depth1 foraging */
  void visit(fsm::depth1::block_to_goal_fsm& fsm) override;
  void visit(tasks::depth1::harvester& task) override;
  void visit(
      controller::depth1::greedy_partitioning_controller& controller) override;

  /* depth2 foraging */
  void visit(controller::depth2::greedy_recpart_controller& controller) override;
  void visit(tasks::depth2::cache_starter& task) override;
  void visit(tasks::depth2::cache_finisher& task) override;

 private:
  uint m_block_id;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_VANISHED_HPP_ */
