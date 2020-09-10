/**
 * \file robot_nest_block_drop.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_ROBOT_NEST_BLOCK_DROP_HPP_
#define INCLUDE_FORDYCA_EVENTS_ROBOT_NEST_BLOCK_DROP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/repr/base_block3D.hpp"

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
 * \class robot_nest_block_drop
 * \ingroup events detail
 *
 * \brief Fired whenever a robot drops a block in the nest.
 */
class robot_nest_block_drop : public rer::client<robot_nest_block_drop> {
 private:
  struct visit_typelist_impl {
    using controllers = boost::mpl::joint_view<
        boost::mpl::joint_view<controller::d0::typelist,
                               controller::d1::typelist>,
        controller::d2::typelist>;

    using fsms = rmpl::typelist<fsm::d0::crw_fsm,
                                fsm::d0::dpo_fsm,
                                fsm::d0::free_block_to_nest_fsm,
                                fsm::d1::cached_block_to_nest_fsm>;
    using tasks =
        rmpl::typelist<tasks::d0::generalist, tasks::d1::collector>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<controllers::type, tasks::type>,
        fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * \brief Initialize a nest block drop event.
   *
   * \param block The block the robot has just dropped into the nest, owned by
   *              the arena.
   * \param t Current timestep.
   */
  robot_nest_block_drop(crepr::base_block3D* block, const rtypes::timestep& t);

  ~robot_nest_block_drop(void) override = default;

  robot_nest_block_drop(const robot_nest_block_drop& op) = delete;
  robot_nest_block_drop& operator=(const robot_nest_block_drop& op) = delete;

  /* Depth0 DPO/MDPO foraging */
  void visit(controller::reactive::d0::crw_controller& controller);
  void visit(controller::cognitive::d0::dpo_controller& controller);
  void visit(controller::cognitive::d0::mdpo_controller& controller);
  void visit(controller::cognitive::d0::odpo_controller& controller);
  void visit(controller::cognitive::d0::omdpo_controller& controller);

  /* Depth1 foraging */
  void visit(controller::cognitive::d1::bitd_dpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_mdpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_odpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_omdpo_controller& controller);
  void visit(tasks::d1::collector& task);
  void visit(tasks::d0::generalist& task);

  /* d2 foraging */
  void visit(controller::cognitive::d2::birtd_dpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_mdpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_odpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_omdpo_controller& controller);

 private:
  void visit(fsm::d0::free_block_to_nest_fsm& fsm);
  void visit(fsm::d1::cached_block_to_nest_fsm& fsm);
  void visit(fsm::d0::dpo_fsm& fsm);
  void visit(fsm::d0::crw_fsm& fsm);

  void dispatch_nest_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  const rtypes::timestep mc_timestep;

  crepr::base_block3D*   m_block;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using robot_nest_block_drop_visitor_impl =
    rpvisitor::precise_visitor<detail::robot_nest_block_drop,
                               detail::robot_nest_block_drop::visit_typelist>;

NS_END(detail);

class robot_nest_block_drop_visitor
    : public detail::robot_nest_block_drop_visitor_impl {
 public:
  using detail::robot_nest_block_drop_visitor_impl::robot_nest_block_drop_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_ROBOT_NEST_BLOCK_DROP_HPP_ */
