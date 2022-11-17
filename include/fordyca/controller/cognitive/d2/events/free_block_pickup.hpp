/**
 * \file free_block_pickup.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/controller/operations/base_block_pickup.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"
#include "fordyca/subsystem/perception/perception_fwd.hpp"
#include "fordyca/controller/cognitive/d1/events/free_block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/

NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_block_pickup
 * \ingroup controller cognitive d2 events
 *
 * \brief Fired whenever a robot picks up a free block in the arena (i.e. one
 * that is not part of a cache).
 */
class free_block_pickup : public rer::client<free_block_pickup>,
                          public fccd1::events::free_block_pickup {
 private:
  struct visit_typelist_impl {
    using controllers = controller::d2::typelist;
    using fsms = rmpl::typelist<ffsm::block_to_goal_fsm,
                                ffsm::d0::free_block_to_nest_fsm>;
    using value = boost::mpl::joint_view<controllers::type, fsms::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  ~free_block_pickup(void) override = default;

  free_block_pickup(const free_block_pickup&) = delete;
  free_block_pickup& operator=(const free_block_pickup&) = delete;


  /* controllers */
  void visit(fccd2::birtd_dpo_controller& controller);
  void visit(fccd2::birtd_mdpo_controller& controller);
  void visit(fccd2::birtd_odpo_controller& controller);
  void visit(fccd2::birtd_omdpo_controller& controller);

  using fccd1::events::free_block_pickup::visit;

 protected:
  free_block_pickup(crepr::sim_block3D* block,
                    const rtypes::type_uuid& id,
                    const rtypes::timestep& t);
 private:
  /*
   * We need our own copy because we use *this and otherwise we get the type of
   * the d1 version of this visitor.
   */
  template<typename TController, typename TPerceptionModel>
  void controller_process(TController& controller, TPerceptionModel& model) {
    fccd0::events::free_block_pickup::visit(model);
    ccops::base_block_pickup::visit(static_cast<ccontroller::block_carrying_controller&>(controller));
    task_dispatch(controller.current_task(), *this);
  }
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using free_block_pickup_visitor = rpvisitor::filtered_visitor<free_block_pickup>;

NS_END(events, d2, cognitive, controller, fordyca);
