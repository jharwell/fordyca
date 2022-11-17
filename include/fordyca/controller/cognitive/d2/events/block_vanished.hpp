/**
 * \file block_vanished.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/controller/cognitive/d1/events/block_vanished.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_vanished
 * \ingroup controller cognitive d2 events
 *
 * \brief Created whenever a robot is serving a block pickup penalty, but while
 * serving the penalty the block it is waiting for vanishes due to another
 * robot picking it up.
 */
class block_vanished : public rer::client<block_vanished>,
                       public fccd1::events::block_vanished {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d2::typelist;
    using fsms = rmpl::typelist<ffsm::block_to_goal_fsm>;

    using value = boost::mpl::joint_view<controllers::type, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;
  using fccd1::events::block_vanished::visit;

  explicit block_vanished(const rtypes::type_uuid& block_id);
  ~block_vanished(void) override = default;

  block_vanished(const block_vanished&) = delete;
  block_vanished& operator=(const block_vanished&) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& controller);
  void visit(fccd2::birtd_mdpo_controller& controller);
  void visit(fccd2::birtd_odpo_controller& controller);
  void visit(fccd2::birtd_omdpo_controller& controller);


 private:
  void dispatch_free_block_interactor(ftasks::base_foraging_task* task);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_vanished_visitor = rpvisitor::filtered_visitor<block_vanished>;

NS_END(events, d2, cognitive, controller, fordyca);

