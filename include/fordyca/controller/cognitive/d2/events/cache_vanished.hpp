/**
 * \file cache_vanished.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/cognitive/d1/events/cache_vanished.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_vanished
 * \ingroup controller cognitive d2 events
 *
 * \brief Created whenever a robot is serving a cache penalty, but while
 * serving the penalty the cache it is waiting in vanishes due to another
 * robot picking up the last available block.
 */
class cache_vanished : public rer::client<cache_vanished>,
                       public fccd1::events::cache_vanished {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d2::cognitive_typelist;
    using fsms = rmpl::typelist<fsm::block_to_goal_fsm>;
    using value = boost::mpl::joint_view<fsms::type, controllers::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;
  using fccd1::events::cache_vanished::visit;

  explicit cache_vanished(const rtypes::type_uuid& cache_id);
  ~cache_vanished(void) override = default;

  cache_vanished(const cache_vanished& op) = delete;
  cache_vanished& operator=(const cache_vanished& op) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& controller);
  void visit(fccd2::birtd_mdpo_controller& controller);
  void visit(fccd2::birtd_odpo_controller& controller);
  void visit(fccd2::birtd_omdpo_controller& controller);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_vanished_visitor = rpvisitor::filtered_visitor<cache_vanished>;

NS_END(events, d2, cognitive, controller, fordyca);
