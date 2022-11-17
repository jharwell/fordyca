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
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cache_vanished
 * \ingroup controller cognitive d1 events
 *
 * \brief Created whenever a robot is serving a cache penalty, but while
 * serving the penalty the cache it is waiting in vanishes due to another
 * robot picking up the last available block.
 */
class cache_vanished : public rer::client<cache_vanished> {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d1::cognitive_typelist;
    using fsms = rmpl::typelist<ffsm::block_to_goal_fsm,
                                ffsm::d1::cached_block_to_nest_fsm>;
    using value = boost::mpl::joint_view<controllers::type, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;
  explicit cache_vanished(const rtypes::type_uuid& cache_id);
  ~cache_vanished(void) override = default;

  cache_vanished(const cache_vanished& op) = delete;
  cache_vanished& operator=(const cache_vanished& op) = delete;

  /* controllers */
  void visit(fccd1::bitd_dpo_controller& controller);
  void visit(fccd1::bitd_mdpo_controller& controller);
  void visit(fccd1::bitd_odpo_controller& controller);
  void visit(fccd1::bitd_omdpo_controller& controller);

  /* FSMs */
  void visit(ffsm::d1::cached_block_to_nest_fsm& fsm);
  void visit(ffsm::block_to_goal_fsm& fsm);

 protected:
  const rtypes::type_uuid& cache_id(void) const { return mc_cache_id; }
  void dispatch_cache_interactor(ftasks::base_foraging_task* task);

 private:
  /* clang-format off */
    const rtypes::type_uuid mc_cache_id;
  /* clang-format on */
};


/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using cache_vanished_visitor = rpvisitor::filtered_visitor<cache_vanished>;

NS_END(events, d1, cognitive, controller, fordyca);
