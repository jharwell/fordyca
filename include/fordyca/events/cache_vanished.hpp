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
#include "fordyca/events/cell_op.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace visitor = rcppsw::patterns::visitor;
namespace controller {
namespace depth1 {
class gp_dpo_controller;
class gp_mdpo_controller;
} // namespace depth1
namespace depth2 {
class grp_dpo_controller;
class grp_mdpo_controller;
} // namespace depth2
} // namespace controller

namespace fsm {
namespace depth1 {
class cached_block_to_nest_fsm;
}
class block_to_goal_fsm;
} // namespace fsm
namespace tasks {
class base_foraging_task;
namespace depth1 {
class collector;
class harvester;
} // namespace depth1
namespace depth2 {
class cache_transferer;
}
} // namespace tasks

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct cache_vanished_visit_set {
  using value =
      visitor::precise_visit_set<controller::depth1::gp_dpo_controller,
                                 controller::depth1::gp_mdpo_controller,
                                 controller::depth2::grp_dpo_controller,
                                 controller::depth2::grp_mdpo_controller,
                                 tasks::depth1::collector,
                                 tasks::depth1::harvester,
                                 tasks::depth2::cache_transferer,
                                 fsm::block_to_goal_fsm,
                                 fsm::depth1::cached_block_to_nest_fsm>;
};

/*
 * @class cache_vanished
 * @ingroup events detail
 *
 * @brief Created whenever a robot is serving a cache penalty, but while
 * serving the penalty the cache it is waiting in vanishes due to another
 * robot picking up the last available block.
 */
class cache_vanished : public rcppsw::er::client<cache_vanished> {
 public:
  explicit cache_vanished(uint cache_id);
  ~cache_vanished(void) override = default;

  cache_vanished(const cache_vanished& op) = delete;
  cache_vanished& operator=(const cache_vanished& op) = delete;

  /* depth1 foraging */
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(fsm::depth1::cached_block_to_nest_fsm& fsm);
  void visit(tasks::depth1::collector& task);
  void visit(tasks::depth1::harvester& task);
  void visit(controller::depth1::gp_dpo_controller& controller);
  void visit(controller::depth1::gp_mdpo_controller& controller);

  /* depth2 foraging */
  void visit(controller::depth2::grp_dpo_controller& controller);
  void visit(controller::depth2::grp_mdpo_controller& controller);
  void visit(tasks::depth2::cache_transferer& controller);

 private:
  /* clang-format off */
  void dispatch_cache_interactor(tasks::base_foraging_task* task);
  uint m_cache_id;
  /* clang-format on */
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_vanished_visitor_impl =
    visitor::precise_visitor<detail::cache_vanished,
                             detail::cache_vanished_visit_set::value>;

NS_END(detail);

class cache_vanished_visitor : public detail::cache_vanished_visitor_impl {
  using detail::cache_vanished_visitor_impl::cache_vanished_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_VANISHED_HPP_ */
