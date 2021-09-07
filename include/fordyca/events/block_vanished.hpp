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

#ifndef INCLUDE_FORDYCA_EVENTS_BLOCK_VANISHED_HPP_
#define INCLUDE_FORDYCA_EVENTS_BLOCK_VANISHED_HPP_

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
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_vanished
 * \ingroup events detail
 *
 * \brief Created whenever a robot is serving a block pickup penalty, but while
 * serving the penalty the block it is waiting for vanishes due to another
 * robot picking it up (ramp blocks only).
 */
class block_vanished : public rer::client<block_vanished> {
 private:
  struct visit_typelist_impl {
    using controllers = boost::mpl::joint_view<
        boost::mpl::joint_view<controller::d0::typelist, controller::d1::typelist>,
        controller::d2::typelist>;
    using tasks = rmpl::typelist<tasks::d0::generalist,
                                 tasks::d1::harvester,
                                 tasks::d2::cache_starter,
                                 tasks::d2::cache_finisher>;
    using fsms = rmpl::typelist<fsm::d0::crw_fsm,
                                fsm::d0::dpo_fsm,
                                fsm::d0::free_block_to_nest_fsm,
                                fsm::block_to_goal_fsm>;

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

  /* d0 foraging */
  void visit(controller::reactive::d0::crw_controller& controller);
  void visit(controller::cognitive::d0::dpo_controller& controller);
  void visit(controller::cognitive::d0::mdpo_controller& controller);
  void visit(controller::cognitive::d0::odpo_controller& controller);
  void visit(controller::cognitive::d0::omdpo_controller& controller);
  void visit(tasks::d0::generalist& task);
  void visit(fsm::d0::crw_fsm& fsm);
  void visit(fsm::d0::dpo_fsm& fsm);

  /* d1 foraging */
  void visit(fsm::d0::free_block_to_nest_fsm& fsm);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(tasks::d1::harvester& task);
  void visit(controller::cognitive::d1::bitd_dpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_mdpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_odpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_omdpo_controller& controller);

  /* d2 foraging */
  void visit(controller::cognitive::d2::birtd_dpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_mdpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_odpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_omdpo_controller& controller);
  void visit(tasks::d2::cache_starter& task);
  void visit(tasks::d2::cache_finisher& task);

 private:
  void dispatch_free_block_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  const rtypes::type_uuid mc_block_id;
  /* clang-format on */
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_vanished_visitor =
    rpvisitor::filtered_visitor<detail::block_vanished>;

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_BLOCK_VANISHED_HPP_ */
