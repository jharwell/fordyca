/**
 * @file oracular_partitioning_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH1_ORACULAR_PARTITIONING_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH1_ORACULAR_PARTITIONING_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/controller/depth1/greedy_partitioning_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace support {
class tasking_oracle;
}

NS_START(controller, depth1);


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class oracular_partitioning_controller
 * @ingroup controller depth1
 *
 * @brief A greedy partitioning controller which also has perfect information
 * about:
 *
 * - Average task durations
 *
 * for use in task allocation.
 */
class oracular_partitioning_controller : public greedy_partitioning_controller,
                                         public er::client<oracular_partitioning_controller>,
                                         public visitor::visitable_any<oracular_partitioning_controller> {
 public:
  oracular_partitioning_controller(void)
      : ER_CLIENT_INIT("fordyca.controller.depth1.oracular_partitioning") {}

  ~oracular_partitioning_controller(void) override = default;
  oracular_partitioning_controller& operator=(
      const oracular_partitioning_controller& other);
  oracular_partitioning_controller(
      const oracular_partitioning_controller& other);

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;

  /**
   * @brief Set the tasking oracle for the controller. Should only be called
   * once, during initialization.
   */
  void tasking_oracle(support::tasking_oracle*oracle) {
    mc_tasking_oracle = oracle;
  }

 private:
  /**
   * @brief Uses the \ref support::tasking_oracle to update the execution time
   * estimate for the task that was just aborted.
   */
  void task_abort_cb(ta::polled_task* task);

  /**
   * @brief Uses the \ref support::tasking_oracle to update the execution time
   * estimate for the task that was just finished.
   */
  void task_finish_cb(ta::polled_task* task);

  // clang-format off
  const support::tasking_oracle* mc_tasking_oracle{nullptr};
  // clang-format on
};

NS_END(depth1, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH1_ORACULAR_PARTITIONING_CONTROLLER_HPP_ */
