/**
 * @file grp_mdpo_controller.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_MDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_MDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth1/gp_mdpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw { namespace task_allocation {
class bi_tdgraph_executive;
}}

NS_START(fordyca);
namespace visitor = rcppsw::patterns::visitor;
namespace ta = rcppsw::task_allocation;

namespace tasks { namespace depth2 {
class foraging_task;
}}
namespace params { namespace depth2 { class controller_repository; }}
NS_START(controller, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class grp_mdpo_controller
 * @ingroup controller depth2
 *
 * @brief A Greedy Recursive Partitioning controller that moves through a depth2
 * recursive task decomposition graph, changing task according to dynamic
 * changes in the environment and/or execution/interface times of the tasks, and
 * using a Mapped DPO data store for tracking arena state and object relevance.
 */
class grp_mdpo_controller : public depth1::gp_mdpo_controller,
                                  public er::client<grp_mdpo_controller>,
                                  public visitor::visitable_any<grp_mdpo_controller> {
 public:
  grp_mdpo_controller(void);
  ~grp_mdpo_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;

  void bsel_exception_added(bool b) { m_bsel_exception_added = b; }
  void csel_exception_added(bool b) { m_csel_exception_added = b; }

  void shared_init(const params::depth2::controller_repository& param_repo);

 private:
  void task_alloc_cb(const ta::polled_task* const task,
                     const ta::bi_tab* const);

  /* clang-format off */
  /**
   * @brief \c TRUE if the controller's most recently completed task involved
   * the dropping of a free block (i.e. culminated in a \ref free_block_drop).
   * Needed so that if the robot's next task requires picking up a free block
   * that the robot does not pick up the same block it just dropped.
   */
  bool m_bsel_exception_added{false};
  bool m_csel_exception_added{false};
  /* clang-format on */
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GRP_MDPO_CONTROLLER_HPP_ */
