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

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d0, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_vanished
 * \ingroup controller reactive d0 events
 *
 * \brief Created whenever a robot is serving a block pickup penalty, but while
 * serving the penalty the block it is waiting for vanishes due to another
 * robot picking it up.
 */
class block_vanished : public rer::client<block_vanished> {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d0::cognitive_typelist;
    using fsms = rmpl::typelist<ffsm::d0::dpo_fsm,
                                ffsm::d0::free_block_to_nest_fsm>;
    using tasks = rmpl::typelist<ftasks::d0::generalist>;

    using tmp = boost::mpl::joint_view<controllers, fsms::type>;
    using value = boost::mpl::joint_view<tmp, tasks::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_vanished(const rtypes::type_uuid& block_id);
  ~block_vanished(void) override = default;

  block_vanished(const block_vanished&) = delete;
  block_vanished& operator=(const block_vanished&) = delete;

  /* controllers */
  void visit(fccd0::dpo_controller& controller);
  void visit(fccd0::mdpo_controller& controller);
  void visit(fccd0::odpo_controller& controller);
  void visit(fccd0::omdpo_controller& controller);

  /* FSMs */
  void visit(ffsm::d0::free_block_to_nest_fsm& fsm);

 protected:
  const rtypes::type_uuid& block_id(void) const { return mc_block_id; }

 private:
  /* FSMs */
  void visit(ffsm::d0::crw_fsm& fsm);
  void visit(ffsm::d0::dpo_fsm& fsm);

  /* clang-format off */
  const rtypes::type_uuid mc_block_id;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_vanished_visitor = rpvisitor::filtered_visitor<block_vanished>;

NS_END(events, d0, cognitive, controller, fordyca);
