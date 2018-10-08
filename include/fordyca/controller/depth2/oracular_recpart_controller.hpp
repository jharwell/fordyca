/**
 * @file oracular_recpart_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_ORACULAR_RECPART_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_ORACULAR_RECPART_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/controller/depth1/oracular_partitioning_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace support {
class tasking_oracle;
}

NS_START(controller, depth2);


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class oracular_recpart_controller
 * @ingroup controller depth2
 *
 * @brief A greedy recpart controller which also has perfect information
 * about:
 *
 * - Average task durations
 *
 * for use in task allocation.
 */
class oracular_recpart_controller : public depth1::oracular_partitioning_controller,
                                    public er::client<oracular_recpart_controller>,
                                    public visitor::visitable_any<oracular_recpart_controller> {
 public:
  oracular_recpart_controller(void)
      : ER_CLIENT_INIT("fordyca.controller.depth2.oracular_recpart") {}
  ~oracular_recpart_controller(void) override = default;

  oracular_recpart_controller& operator=(
      const oracular_recpart_controller& other);
  oracular_recpart_controller(
      const oracular_recpart_controller& other);

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_ORACULAR_RECPART_CONTROLLER_HPP_ */
