/**
 * @file stateful_tasking_initializer.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATEFUL_TASKING_INITIALIZER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATEFUL_TASKING_INITIALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace rcppsw {
namespace er { class server; }
namespace task_allocation {
class polled_executive;
class task_decomposition_graph;
}}
namespace ta = rcppsw::task_allocation;

NS_START(fordyca);
namespace params {
namespace depth0 { class stateful_param_repository; }
}

NS_START(controller);
class saa_subsystem;
class base_perception_subsystem;

NS_START(depth0);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class stateful_tasking_initializer
 * @ingroup controller depth0
 *
 * @brief A helper class to offload initialization of the task tree for depth0
 * foraging.
 */
class stateful_tasking_initializer {
 public:
  stateful_tasking_initializer(std::shared_ptr<rcppsw::er::server> server,
                               saa_subsystem* saa,
                               base_perception_subsystem* perception);
  virtual ~stateful_tasking_initializer(void);
  stateful_tasking_initializer& operator=(const stateful_tasking_initializer& other) = delete;
  stateful_tasking_initializer(const stateful_tasking_initializer& other) = delete;

  std::unique_ptr<ta::polled_executive>
  operator()(params::depth0::stateful_param_repository *const stateful_repo);

 protected:
  void stateful_tasking_init(params::depth0::stateful_param_repository* stateful_repo);
  const std::shared_ptr<rcppsw::er::server>& server(void) const { return m_server; }
  const base_perception_subsystem* perception(void) const { return m_perception; }
  base_perception_subsystem* perception(void) { return m_perception; }

  controller::saa_subsystem* saa_subsystem(void) const { return m_saa; }
  std::unique_ptr<ta::task_decomposition_graph>& graph(void) { return m_graph; }

 private:
  // clang-format off
  std::shared_ptr<rcppsw::er::server>           m_server;
  controller::saa_subsystem* const              m_saa;
  base_perception_subsystem* const              m_perception;
  std::unique_ptr<ta::task_decomposition_graph> m_graph;
  // clang-format on
};

NS_END(depth0, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH0_STATEFUL_TASKING_INITIALIZER_HPP_ */
