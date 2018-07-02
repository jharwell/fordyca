/**
 * @file tasking_initializer.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "fordyca/controller/depth0/stateful_tasking_initializer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace ta = rcppsw::task_allocation;
NS_START(fordyca);
namespace params {
namespace depth1 { class param_repository; }
}

NS_START(controller, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class tasking_initializer
 * @ingroup controller depth1
 *
 * @brief A helper class to offload initialization of the task tree for depth1
 * foraging.
 */
class tasking_initializer : public depth0::stateful_tasking_initializer {
 public:
  tasking_initializer(std::shared_ptr<rcppsw::er::server>& server,
                      controller::saa_subsystem* saa,
                      base_perception_subsystem* perception);
  ~tasking_initializer(void);

  std::unique_ptr<ta::polled_executive>
  operator()(params::depth1::param_repository *const param_repo);

 protected:
  void depth1_tasking_init(params::depth1::param_repository* task_repo);
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_ */
