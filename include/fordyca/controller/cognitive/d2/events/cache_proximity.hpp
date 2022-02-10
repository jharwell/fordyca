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

#pragma once

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

NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_proximity
 * \ingroup controller cognitive d2 events
 *
 * \brief Created whenever a robot is attempting to start a new cache, but an
 * existing cache unknown to the robot is too close.
 */
class cache_proximity : public rer::client<cache_proximity> {
 private:
  struct visit_typelist_impl {
    using others = rmpl::typelist<ftasks::d2::cache_finisher,
                                  ftasks::d2::cache_starter,
                                  ffsm::block_to_goal_fsm>;
    using value = boost::mpl::joint_view<fcontroller::d2::typelist::type,
                                         others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit cache_proximity(carepr::base_cache* cache);
  ~cache_proximity(void) override = default;

  cache_proximity(const cache_proximity&) = delete;
  cache_proximity& operator=(const cache_proximity&) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& c);
  void visit(fccd2::birtd_odpo_controller& c);
  void visit(fccd2::birtd_mdpo_controller& c);
  void visit(fccd2::birtd_omdpo_controller& c);

  /* tasks */
  void visit(tasks::d2::cache_finisher& task);
  void visit(tasks::d2::cache_starter& task);

 private:
  /* FSMs */
  void visit(fsm::block_to_goal_fsm& fsm);

  void dispatch_cache_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  carepr::base_cache* m_cache;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_proximity_visitor = rpvisitor::filtered_visitor<cache_proximity>;

NS_END(events, d2, cognitive, controller, fordyca);

