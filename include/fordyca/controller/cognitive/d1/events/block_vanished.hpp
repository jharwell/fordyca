/**
 * \file block_vanished.hpp
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
 * \class block_vanished
 * \ingroup controller cognitive d1 events
 *
 * \brief Created whenever a robot is serving a block pickup penalty, but while
 * serving the penalty the block it is waiting for vanishes due to another
 * robot picking it up.
 */
class block_vanished : public rer::client<block_vanished> {
 private:
  struct visit_typelist_impl {
    using controllers = fcontroller::d1::cognitive_typelist;
    using tasks = rmpl::typelist<ftasks::d1::harvester>;

    using fsms = rmpl::typelist<ffsm::block_to_goal_fsm>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<controllers::type, tasks::type>,
        fsms::type>;
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

  /* tasks */
  void visit(ftasks::d1::harvester& task);

 protected:
  void dispatch_free_block_interactor(ftasks::base_foraging_task* const task);
  const rtypes::type_uuid& block_id(void) const { return mc_block_id; }

  /* FSMs */
  void visit(ffsm::block_to_goal_fsm& fsm);

 private:
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

NS_END(events, d1, reactive, controller, fordyca);

