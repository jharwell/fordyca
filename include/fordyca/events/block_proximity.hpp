/**
 * @file block_proximity.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_PROXIMITY_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_PROXIMITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

namespace repr {
class base_block;
}

NS_START(events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class block_proximity
 * @ingroup fordyca events
 *
 * @brief Event that is created whenever a block that a robot is not currently
 * aware of blocks its ability to complete its current task.
 */
class block_proximity : public rer::client<block_proximity> {
 private:
  struct visit_typelist_impl {
    using controllers = controller::depth2::typelist;
    using others =
        rmpl::typelist<fsm::block_to_goal_fsm, tasks::depth2::cache_starter>;
    using value = boost::mpl::joint_view<controllers, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_proximity(const std::shared_ptr<repr::base_block>& block);
  ~block_proximity(void) override = default;

  block_proximity(const block_proximity& op) = delete;
  block_proximity& operator=(const block_proximity& op) = delete;

  /* depth2 foraging */
  void visit(controller::depth2::birtd_dpo_controller& c);
  void visit(controller::depth2::birtd_mdpo_controller& c);
  void visit(controller::depth2::birtd_odpo_controller& c);
  void visit(controller::depth2::birtd_omdpo_controller& c);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(tasks::depth2::cache_starter& task);

 private:
  void dispatch_cache_starter(tasks::base_foraging_task* task);

  /* clang-format off */
  std::shared_ptr<repr::base_block> m_block;
  /* clang-format on */
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_proximity_visitor_impl =
    rpvisitor::precise_visitor<detail::block_proximity,
                               detail::block_proximity::visit_typelist>;

NS_END(detail);

class block_proximity_visitor : public detail::block_proximity_visitor_impl {
  using detail::block_proximity_visitor_impl::block_proximity_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_PROXIMITY_HPP_ */
