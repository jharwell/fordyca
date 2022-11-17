/**
 * \file cache_proximity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
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
    using others = rmpl::typelist<ffsm::block_to_goal_fsm>;
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

  /* FSMs */
  void visit(fsm::block_to_goal_fsm& fsm);

 private:
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
