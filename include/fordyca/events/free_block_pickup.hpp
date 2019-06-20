/**
 * @file free_block_pickup.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/events/block_pickup_base_visit_set.hpp"
#include "fordyca/events/cell_op.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"
#include "rcppsw/er/client.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class free_block_pickup
 * @ingroup fordyca events detail
 *
 * @brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache).
 */
class free_block_pickup : public rer::client<free_block_pickup>, public cell_op {
 private:
  struct visit_typelist_impl {
    using inherited = boost::mpl::joint_view<block_pickup_base_visit_typelist,
                                             cell_op::visit_typelist>;
    using controllers = boost::mpl::joint_view<
        boost::mpl::joint_view<controller::depth0::typelist,
                               controller::depth1::typelist>,
        controller::depth2::typelist>;
    using tasks = rmpl::typelist<tasks::depth0::generalist,
                                 tasks::depth1::harvester,
                                 tasks::depth2::cache_starter,
                                 tasks::depth2::cache_finisher>;
    using fsms = rmpl::typelist<fsm::depth0::crw_fsm,
                                fsm::depth0::dpo_fsm,
                                fsm::depth0::free_block_to_nest_fsm,
                                fsm::block_to_goal_fsm>;
    using value = boost::mpl::joint_view<
        boost::mpl::joint_view<boost::mpl::joint_view<controllers::type, tasks::type>,
                               fsms::type>,
        boost::mpl::joint_view<inherited::type, controllers::type> >;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  free_block_pickup(const std::shared_ptr<repr::base_block>& block,
                    uint robot_index,
                    uint timestep);
  ~free_block_pickup(void) override = default;

  free_block_pickup(const free_block_pickup& op) = delete;
  free_block_pickup& operator=(const free_block_pickup& op) = delete;

  /* CRW foraging */
  void visit(ds::arena_map& map);
  void visit(ds::cell2D& cell);
  void visit(fsm::cell2D_fsm& fsm);
  void visit(repr::base_block& block);
  void visit(controller::depth0::crw_controller& controller);
  void visit(fsm::depth0::crw_fsm& fsm);

  /* Depth0 DPO/MDPO foraging */
  void visit(ds::dpo_store& store);
  void visit(ds::dpo_semantic_map& map);
  void visit(fsm::depth0::dpo_fsm& fsm);
  void visit(controller::depth0::dpo_controller& controller);
  void visit(controller::depth0::mdpo_controller& controller);
  void visit(controller::depth0::odpo_controller& controller);
  void visit(controller::depth0::omdpo_controller& controller);

  /* depth1 DPO/MDPO foraging */
  void visit(fsm::depth0::free_block_to_nest_fsm& fsm);
  void visit(controller::depth1::gp_dpo_controller& controller);
  void visit(controller::depth1::gp_mdpo_controller& controller);
  void visit(controller::depth1::gp_odpo_controller& controller);
  void visit(controller::depth1::gp_omdpo_controller& controller);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(tasks::depth0::generalist& task);
  void visit(tasks::depth1::harvester& task);

  /* depth2 DPO/MDPO foraging */
  void visit(controller::depth2::grp_dpo_controller& controller);
  void visit(controller::depth2::grp_mdpo_controller& controller);
  void visit(controller::depth2::grp_odpo_controller& controller);
  void visit(controller::depth2::grp_omdpo_controller& controller);
  void visit(tasks::depth2::cache_starter& task);
  void visit(tasks::depth2::cache_finisher& task);

 private:
  void dispatch_free_block_interactor(tasks::base_foraging_task* task);

  /* clang-format off */
  uint                              m_timestep;
  uint                              m_robot_index;
  std::shared_ptr<repr::base_block> m_block;
  /* clang-format on */
};

/**
 * @brief We use the picky visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using free_block_pickup_visitor_impl =
    rpvisitor::precise_visitor<detail::free_block_pickup,
                               detail::free_block_pickup::visit_typelist>;

NS_END(detail);

class free_block_pickup_visitor : public detail::free_block_pickup_visitor_impl {
  using detail::free_block_pickup_visitor_impl::free_block_pickup_visitor_impl;
};

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_FREE_BLOCK_PICKUP_HPP_ */
