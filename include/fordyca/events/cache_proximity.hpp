/**
 * \file cache_proximity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_CACHE_PROXIMITY_HPP_
#define INCLUDE_FORDYCA_EVENTS_CACHE_PROXIMITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::repr {
class base_cache;
}

NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_proximity
 * \ingroup events detail
 *
 * \brief Created whenever a robot is attempting to start a new cache, but an
 * existing cache unknown to the robot is too close.
 */
class cache_proximity : public rer::client<cache_proximity> {
 private:
  struct visit_typelist_impl {
    using others = rmpl::typelist<tasks::depth2::cache_finisher,
                                  tasks::depth2::cache_starter,
                                  fsm::block_to_goal_fsm>;
    using value =
        boost::mpl::joint_view<controller::depth2::typelist::type, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cache_proximity(carepr::base_cache* cache);
  ~cache_proximity(void) override = default;

  cache_proximity(const cache_proximity& op) = delete;
  cache_proximity& operator=(const cache_proximity& op) = delete;

  /* depth2 foraging */
  void visit(controller::depth2::birtd_dpo_controller& c);
  void visit(controller::depth2::birtd_odpo_controller& c);
  void visit(controller::depth2::birtd_mdpo_controller& c);
  void visit(controller::depth2::birtd_omdpo_controller& c);
  void visit(tasks::depth2::cache_finisher& task);
  void visit(tasks::depth2::cache_starter& task);
  void visit(fsm::block_to_goal_fsm& fsm);

 private:
  void dispatch_cache_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  carepr::base_cache* m_cache;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_proximity_visitor_impl =
    rpvisitor::precise_visitor<detail::cache_proximity,
                               detail::cache_proximity::visit_typelist>;

NS_END(detail);

class cache_proximity_visitor : public detail::cache_proximity_visitor_impl {
  using detail::cache_proximity_visitor_impl::cache_proximity_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_CACHE_PROXIMITY_HPP_ */
