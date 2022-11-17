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
#include "fordyca/controller/cognitive/d0/events/block_vanished.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_vanished
 * \ingroup controller cognitive d1 events
 *
 * \brief Created whenever a robot is serving a block pickup penalty, but while
 * serving the penalty the block it is waiting for vanishes due to another
 * robot picking it up.
 */
class block_vanished : public rer::client<block_vanished>,
                       public fccd0::events::block_vanished {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d1::cognitive_typelist;

    using fsms = rmpl::typelist<ffsm::block_to_goal_fsm,
                                ffsm::d0::free_block_to_nest_fsm>;

    using value = boost::mpl::joint_view<controllers::type, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_vanished(const rtypes::type_uuid& block_id);
  ~block_vanished(void) override = default;

  block_vanished(const block_vanished&) = delete;
  block_vanished& operator=(const block_vanished&) = delete;

  /* controllers */
  void visit(fccognitive::d1::bitd_dpo_controller& controller);
  void visit(fccognitive::d1::bitd_mdpo_controller& controller);
  void visit(fccognitive::d1::bitd_odpo_controller& controller);
  void visit(fccognitive::d1::bitd_omdpo_controller& controller);

  /* FSMs */
  void visit(ffsm::block_to_goal_fsm& fsm);

 protected:
  void dispatch_free_block_interactor(ftasks::base_foraging_task* const task);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_vanished_visitor = rpvisitor::filtered_visitor<block_vanished>;

NS_END(events, d1, reactive, controller, fordyca);
