/**
 * @file ogrp_mdpo_controller.hpp
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

#ifndef INCLUDE_FORDYCA_CONTROLLER_DEPTH2_OGRP_MDPO_CONTROLLER_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_DEPTH2_OGRP_MDPO_CONTROLLER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "fordyca/controller/depth1/ogp_mdpo_controller.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace support {
class tasking_oracle;
}
namespace params { namespace depth2 { class controller_repository; }}

NS_START(controller, depth2);


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class ogrp_mdpo_controller
 * @ingroup fordyca controller depth2
 *
 * @brief A Greedy Recursive Partitioning controller which also has perfect
 * information about:
 *
 * - Average task durations
 *
 * for use in task allocation.
 */
class ogrp_mdpo_controller : public depth1::ogp_mdpo_controller,
                             public rer::client<ogrp_mdpo_controller> {
 public:
  ogrp_mdpo_controller(void)
      : ER_CLIENT_INIT("fordyca.controller.depth2.ogrp_mdpo") {}
  ~ogrp_mdpo_controller(void) override = default;

  /* CCI_Controller overrides */
  void Init(ticpp::Element& node) override;

  void shared_init(const params::depth2::controller_repository& param_repo);
};

NS_END(depth2, controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_DEPTH2_OGRP_MDPO_CONTROLLER_HPP_ */
