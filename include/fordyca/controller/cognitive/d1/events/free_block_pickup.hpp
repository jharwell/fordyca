/**
 * \file free_block_pickup.hpp
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

#pragma once

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
#include "fordyca/controller/cognitive/d0/events/free_block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/

NS_START(fordyca, controller, cognitive, d1, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_pickup
 * \ingroup controller cognitive d1 events
 *
 * \brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache).
 */
class free_block_pickup : public rer::client<free_block_pickup>,
                          public fccd0::events::free_block_pickup {
 private:
  struct visit_typelist_impl {
    using controllers = controller::d1::typelist;
    using tasks = rmpl::typelist<tasks::d1::harvester>;
    using fsms = rmpl::typelist<fsm::block_to_goal_fsm>;
    using value1 = boost::mpl::joint_view<controllers::type, tasks::type>;
    using value = boost::mpl::joint_view<value1, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~free_block_pickup(void) override = default;

  free_block_pickup(const free_block_pickup&) = delete;
  free_block_pickup& operator=(const free_block_pickup&) = delete;

  /* controllers */
  void visit(fccognitive::d1::bitd_dpo_controller& controller);
  void visit(fccognitive::d1::bitd_mdpo_controller& controller);
  void visit(fccognitive::d1::bitd_odpo_controller& controller);
  void visit(fccognitive::d1::bitd_omdpo_controller& controller);

  /* tasks */
  void visit(tasks::d1::harvester& task);

  /* FSMs */
  void visit(fsm::block_to_goal_fsm& fsm);

 protected:
  free_block_pickup(crepr::sim_block3D* block,
                    const rtypes::type_uuid& id,
                    const rtypes::timestep& t);

  template<typename TController, typename TPerceptionModel>
  void controller_visit_impl(TController& controller, TPerceptionModel& model) {
    fccd0::events::free_block_pickup::visit(model);
    ccops::base_block_pickup::visit(static_cast<ccontroller::block_carrying_controller&>(controller));
    dispatch_free_block_interactor(controller.current_task());
  }

 private:
  void dispatch_free_block_interactor(tasks::base_foraging_task* task);
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using free_block_pickup_visitor = rpvisitor::filtered_visitor<free_block_pickup>;

NS_END(events, d1, cognitive, controller, fordyca);

