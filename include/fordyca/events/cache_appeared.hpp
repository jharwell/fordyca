/**
 * @file cache_appeared.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHE_APPEARED_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHE_APPEARED_HPP_

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
namespace controller { namespace depth2 {
class greedy_recpart_controller;
}} // namespace controller::depth2

namespace fsm { namespace depth1 {
class block_to_goal_fsm;
}} // namespace fsm::depth1
namespace tasks { namespace depth2 {
class cache_starter;
class cache_finisher;
}} // namespace tasks::depth2

NS_START(events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/*
 * @class cache_appeared
 * @ingroup events
 *
 * @brief Created whenever a robot is serving a cache penalty, but while
 * serving the penalty the cache it is waiting in vanishes due to another
 * robot picking up the last available block.
 */
class cache_appeared
    : public rcppsw::er::client<cache_appeared>,
      public visitor::visit_set<controller::depth2::greedy_recpart_controller,
                                tasks::depth2::cache_starter,
                                tasks::depth2::cache_finisher,
                                fsm::depth1::block_to_goal_fsm> {
 public:
  explicit cache_appeared(uint cache_id);
  ~cache_appeared(void) override = default;

  cache_appeared(const cache_appeared& op) = delete;
  cache_appeared& operator=(const cache_appeared& op) = delete;

  /* depth2 foraging */
  void visit(controller::depth2::greedy_recpart_controller& controller) override;
  void visit(tasks::depth2::cache_starter& task) override;
  void visit(tasks::depth2::cache_finisher& task) override;
  void visit(fsm::depth1::block_to_goal_fsm& fsm) override;

 private:
  // clang-format off
  uint m_cache_id;
  // clang-format on
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_APPEARED_HPP_ */
