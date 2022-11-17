/**
 * \file free_block_drop.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "fordyca/controller/cognitive/d1/events/free_block_drop.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_drop
 * \ingroup controller cognitive d2 events
 *
 * \brief Created whenever a block is dropped somewhere in the arena that is not
 * a cache or the nest by a robot. Complement to \ref arena_block_drop to handle
 * updates needed by robot controllers.
 *
 * This can happen when:
 *
 * - A robot aborts its task, and is carrying a block.
 */
class free_block_drop : public rer::client<free_block_drop>,
                        public fccd1::events::free_block_drop {
 private:
  struct visit_typelist_impl {
    using inherited = cdops::cell2D_op::visit_typelist;
    using controllers = controller::d2::typelist;

    using others = rmpl::typelist<ffsm::block_to_goal_fsm,
                                  ffsm::d0::free_block_to_nest_fsm>;

    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<controllers::type, others::type>,
        inherited>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~free_block_drop(void) override = default;

  free_block_drop(const free_block_drop& op) = delete;
  free_block_drop& operator=(const free_block_drop& op) = delete;


  /* controllers */
  void visit(fccd2::birtd_dpo_controller& controller);
  void visit(fccd2::birtd_mdpo_controller& controller);
  void visit(fccd2::birtd_odpo_controller& controller);
  void visit(fccd2::birtd_omdpo_controller& controller);

  /* FSMs */
  void visit(ffsm::d0::free_block_to_nest_fsm& fsm);
  void visit(fsm::block_to_goal_fsm& fsm);

 protected:
  /**
   * \param block The block to drop, which was owned by the robot.
   * \param coord The discrete coordinates of the cell to drop the block in.
   * \param resolution The resolution of the arena map.
   */
  free_block_drop(std::unique_ptr<crepr::base_block3D> block,
                        const rmath::vector2z& coord,
                        const rtypes::discretize_ratio& resolution);


 private:

  bool dispatch_free_block_interactor(
      tasks::base_foraging_task* task,
      controller::cognitive::block_sel_matrix* bsel_matrix);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
NS_START(detail);
using free_block_drop_visitor_impl =
    rpvisitor::precise_visitor<free_block_drop,
                               free_block_drop::visit_typelist>;

NS_END(detail);

class free_block_drop_visitor
    : public detail::free_block_drop_visitor_impl {
 public:
  using detail::free_block_drop_visitor_impl::free_block_drop_visitor_impl;
};

NS_END(events, d2, cognitive, controller, fordyca);
