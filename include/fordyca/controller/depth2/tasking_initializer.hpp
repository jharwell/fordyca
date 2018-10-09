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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_TASKING_INITIALIZER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_TASKING_INITIALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "fordyca/controller/depth1/tasking_initializer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace params {
namespace depth2 { class controller_repository; }
}

NS_START(controller, depth2);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class tasking_initializer
 * @ingroup controller depth2
 *
 * @brief A helper class to offload initialization of the task tree for depth2
 * foraging.
 */
class tasking_initializer : public depth1::tasking_initializer,
                            public er::client<tasking_initializer> {
 public:
  tasking_initializer(bool exec_ests_oracle,
                      const controller::block_selection_matrix* bsel_matrix,
                      const controller::cache_selection_matrix* csel_matrix,
                      controller::saa_subsystem* saa,
                      base_perception_subsystem* perception);
  ~tasking_initializer(void) override;

  std::unique_ptr<ta::bi_tdgraph_executive>
  operator()(params::depth2::controller_repository *const param_repo);

 protected:
  void depth2_tasking_init(params::depth2::controller_repository* param_repo);
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_TASKING_INITIALIZER_HPP_ */
