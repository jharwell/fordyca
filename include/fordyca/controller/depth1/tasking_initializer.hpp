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
#include "rcppsw/common/common.hpp"
#include "fordyca/controller/depth0/stateful_tasking_initializer.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace ta = rcppsw::task_allocation;
NS_START(fordyca);
namespace params {
namespace depth1 { class controller_repository; }
}

NS_START(controller);
class cache_selection_matrix;
NS_START(depth1);

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
class tasking_initializer : public depth0::stateful_tasking_initializer,
                            public er::client<tasking_initializer> {
 public:
  tasking_initializer(const controller::block_selection_matrix* bsel_matrix,
                      const controller::cache_selection_matrix* csel_matrix,
                      controller::saa_subsystem* saa,
                      base_perception_subsystem* perception);
  ~tasking_initializer(void);
  tasking_initializer& operator=(const tasking_initializer& other) = delete;
  tasking_initializer(const tasking_initializer& other) = delete;

  std::unique_ptr<ta::bifurcating_tdgraph_executive>
  operator()(params::depth1::controller_repository *const controller_repo);

 protected:
  void depth1_tasking_init(params::depth1::controller_repository* task_repo);
  const cache_selection_matrix* cache_sel_matrix(void) const { return mc_sel_matrix; }

 private:
  // clang-format off
  const controller::cache_selection_matrix* const mc_sel_matrix;
  // clang-format on
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_TASKING_INITIALIZER_HPP_ */
