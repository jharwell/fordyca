/**
 * @file nest_block_drop.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/events/block_drop_event.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;

namespace fsm {
namespace depth0 {
class stateless_foraging_fsm;
class stateful_foraging_fsm;
} // namespace depth0
namespace depth1 {
class block_to_cache_fsm;
class cached_block_to_nest_fsm;
} // namespace depth1
} // namespace fsm
namespace controller { namespace depth0 {
class stateless_foraging_controller;
class stateful_foraging_controller;
}
namespace depth1 { class foraging_controller; }
namespace depth2 { class foraging_controller; }
} // namespace controller

namespace tasks { namespace depth0 {
class generalist;
}
namespace depth1 {
class collector;
}
} // namespace tasks

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class nest_block_drop
 * @ingroup events
 *
 * @brief Fired whenever a robot drops a block in the nest.
 */
class nest_block_drop
    : public visitor::visitor,
      public block_drop_event,
      public rcppsw::er::client,
      public visitor::visit_set<controller::depth0::stateful_foraging_controller,
                                controller::depth0::stateless_foraging_controller,
                                controller::depth1::foraging_controller,
                                controller::depth2::foraging_controller,
                                fsm::depth0::stateless_foraging_fsm,
                                fsm::depth0::stateful_foraging_fsm,
                                fsm::depth1::cached_block_to_nest_fsm,
                                tasks::depth0::generalist,
                                tasks::depth1::collector> {
 public:
  nest_block_drop(const std::shared_ptr<rcppsw::er::server>& server,
                  const std::shared_ptr<representation::block>& block);
  ~nest_block_drop(void) override { client::rmmod(); }

  nest_block_drop(const nest_block_drop& op) = delete;
  nest_block_drop& operator=(const nest_block_drop& op) = delete;

  /* stateless foraging */
  void visit(representation::arena_map& map) override;
  void visit(representation::block& block) override;
  void visit(fsm::depth0::stateless_foraging_fsm& fsm) override;
  void visit(
      controller::depth0::stateless_foraging_controller& controller) override;

  /* stateful foraging */
  void visit(
      controller::depth0::stateful_foraging_controller& controller) override;
  void visit(fsm::depth0::stateful_foraging_fsm& fsm) override;
  void visit(tasks::depth0::generalist& task) override;

  /* depth1 foraging */
  void visit(controller::depth1::foraging_controller& controller) override;
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm) override;
  void visit(tasks::depth1::collector& task) override;

  /* depth2 foraging */
  void visit(controller::depth2::foraging_controller&) override {}

  /**
   * @brief Get the handle on the block that has been dropped.
   */
  std::shared_ptr<representation::block> block(void) const { return m_block; }

 private:
  std::shared_ptr<representation::block> m_block;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_NEST_BLOCK_DROP_HPP_ */
