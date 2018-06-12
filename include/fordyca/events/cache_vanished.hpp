/**
 * @file cache_vanished.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHE_VANISHED_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHE_VANISHED_HPP_

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
namespace depth1 {
class foraging_controller;
}
namespace depth2 {
class foraging_controller;
}
} // namespace controller

namespace fsm { namespace depth1 {
class block_to_goal_fsm;
}} // namespace fsm::depth1
namespace tasks {
namespace depth1 {
class collector;
class harvester;
} // namespace depth1
namespace depth2 {
class cache_transferer;
}
} // namespace tasks

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * @class cache_vanished
 * @ingroup events
 *
 * @brief Created whenever a robot is serving a cache penalty, but while
 * serving the penalty the cache it is waiting in vanishes due to another
 * robot picking up the last available block.
 */
class cache_vanished
    : public rcppsw::er::client,
      public visitor::visit_set<controller::depth1::foraging_controller,
                                controller::depth2::foraging_controller,
                                tasks::depth1::collector,
                                tasks::depth1::harvester,
                                tasks::depth2::cache_transferer,
                                fsm::depth1::block_to_goal_fsm> {
 public:
  cache_vanished(const std::shared_ptr<rcppsw::er::server>& server,
                 uint cache_id);
  ~cache_vanished(void) override { client::rmmod(); }

  cache_vanished(const cache_vanished& op) = delete;
  cache_vanished& operator=(const cache_vanished& op) = delete;

  /* depth1 foraging */
  void visit(fsm::depth1::block_to_goal_fsm& fsm) override;
  void visit(tasks::depth1::collector& task) override;
  void visit(tasks::depth1::harvester& task) override;
  void visit(controller::depth1::foraging_controller& controller) override;

  /* depth2 foraging */
  void visit(controller::depth2::foraging_controller& controller) override;
  void visit(tasks::depth2::cache_transferer& controller) override;

 private:
  uint m_cache_id;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_VANISHED_HPP_ */
