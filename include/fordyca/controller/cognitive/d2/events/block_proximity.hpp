/**
 * \file block_proximity.hpp
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

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"

#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/fordyca.hpp"
#include "fordyca/fsm/fsm_fwd.hpp"
#include "fordyca/tasks/tasks_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

NS_START(fordyca, controller, cognitive, d2, events);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class block_proximity
 * \ingroup controller cognitive d2 events
 *
 * \brief Event that is created whenever a block that a robot is not currently
 * aware of blocks its ability to complete its current task.
 */
class block_proximity : public rer::client<block_proximity> {
 private:
  struct visit_typelist_impl {
    using controllers = controller::d2::typelist;
    using others = rmpl::typelist<ffsm::block_to_goal_fsm>;
    using value = boost::mpl::joint_view<controllers, others::type>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  explicit block_proximity(crepr::sim_block3D* block);
  ~block_proximity(void) override = default;

  block_proximity(const block_proximity& op) = delete;
  block_proximity& operator=(const block_proximity& op) = delete;

  /* controllers */
  void visit(fccd2::birtd_dpo_controller& c);
  void visit(fccd2::birtd_mdpo_controller& c);
  void visit(fccd2::birtd_odpo_controller& c);
  void visit(fccd2::birtd_omdpo_controller& c);

  /* FSMs */
  void visit(fsm::block_to_goal_fsm& fsm);

 private:
  void dispatch_cache_starter(ftasks::base_foraging_task* task);

  /* clang-format off */
  crepr::sim_block3D* m_block;
  /* clang-format on */
};

/**
 * \brief We use the precise visitor in order to force compile errors if a call to
 * a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using block_proximity_visitor = rpvisitor::filtered_visitor<block_proximity>;

NS_END(events, d2, cognitive, controller, fordyca);
