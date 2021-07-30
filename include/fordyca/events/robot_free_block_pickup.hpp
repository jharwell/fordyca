/**
 * \file robot_free_block_pickup.hpp
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

#ifndef INCLUDE_FORDYCA_EVENTS_ROBOT_FREE_BLOCK_PICKUP_HPP_
#define INCLUDE_FORDYCA_EVENTS_ROBOT_FREE_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/controller/operations/base_block_pickup.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/

NS_START(fordyca, events, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_free_block_pickup
 * \ingroup events detail
 *
 * \brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache).
 */
class robot_free_block_pickup : public rer::client<robot_free_block_pickup>,
                                public ccops::base_block_pickup {
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
    using value1 = boost::mpl::joint_view<controllers::type, tasks::type>;
    using value = boost::mpl::joint_view<value1, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~robot_free_block_pickup(void) override = default;

  robot_free_block_pickup(const robot_free_block_pickup&) = delete;
  robot_free_block_pickup& operator=(const robot_free_block_pickup&) = delete;

  /* CRW foraging */
  void visit(controller::reactive::d0::crw_controller& controller);
  void visit(fsm::d0::crw_fsm& fsm);

  /* Depth0 DPO/MDPO foraging */
  void visit(fspds::dpo_store& store);
  void visit(fspds::dpo_semantic_map& map);
  void visit(fsm::d0::dpo_fsm& fsm);
  void visit(controller::cognitive::d0::dpo_controller& controller);
  void visit(controller::cognitive::d0::mdpo_controller& controller);
  void visit(controller::cognitive::d0::odpo_controller& controller);
  void visit(controller::cognitive::d0::omdpo_controller& controller);

  /* d1 DPO/MDPO foraging */
  void visit(fsm::d0::free_block_to_nest_fsm& fsm);
  void visit(controller::cognitive::d1::bitd_dpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_mdpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_odpo_controller& controller);
  void visit(controller::cognitive::d1::bitd_omdpo_controller& controller);
  void visit(fsm::block_to_goal_fsm& fsm);
  void visit(tasks::d0::generalist& task);
  void visit(tasks::d1::harvester& task);

  /* d2 DPO/MDPO foraging */
  void visit(controller::cognitive::d2::birtd_dpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_mdpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_odpo_controller& controller);
  void visit(controller::cognitive::d2::birtd_omdpo_controller& controller);
  void visit(tasks::d2::cache_starter& task);
  void visit(tasks::d2::cache_finisher& task);

 protected:
  robot_free_block_pickup(crepr::base_block3D* block,
                          const rtypes::type_uuid& robot_id,
                          const rtypes::timestep& t);

 private:
  using ccops::base_block_pickup::visit;

  void dispatch_robot_free_block_interactor(tasks::base_foraging_task* task);

  template <typename TController>
  void d1d2_dpo_controller_visit(TController& controller);

  template <typename TController>
  void d1d2_mdpo_controller_visit(TController& controller);

  template <typename TController>
  void d0_dpo_controller_visit(TController& controller);

  template <typename TController>
  void d0_mdpo_controller_visit(TController& controller);
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using robot_free_block_pickup_visitor =
    rpvisitor::filtered_visitor<detail::robot_free_block_pickup>;

NS_END(events, fordyca);

#endif /* INCLUDE_FORDYCA_EVENTS_ROBOT_FREE_BLOCK_PICKUP_HPP_ */
