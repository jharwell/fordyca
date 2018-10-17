/**
 * @file greedy_recpart_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GREEDY_RECPART_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GREEDY_RECPART_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"

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
namespace params { namespace depth2 { class task_repository; }}
NS_START(controller, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class greedy_recpart_controller
 * @ingroup controller depth2
 *
 * @brief A foraging controller that moves through a depth2 recursive task
 * decomposition graph, changing task according to dynamic changes in the
 * environment and/or execution/interface times of the tasks.
 */
class greedy_recpart_controller : public depth1::greedy_partitioning_controller,
                                  public er::client<greedy_recpart_controller>,
                                  public visitor::visitable_any<greedy_recpart_controller> {
 public:
  greedy_recpart_controller(void);
  ~greedy_recpart_controller(void) override;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
  void ControlStep(void) override;


  tasks::base_foraging_task* current_task(void) override;
  const tasks::base_foraging_task* current_task(void) const override;

 private:
  // clang-format off
  std::string                                        m_prev_task{""};
  // clang-format on
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_GREEDY_RECPART_CONTROLLER_HPP_ */