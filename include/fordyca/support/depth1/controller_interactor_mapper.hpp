/**
 * @file controller_interactor_mapper.hpp
 *
 * @copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH1_CONTROLLER_INTERACTOR_MAPPER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH1_CONTROLLER_INTERACTOR_MAPPER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "fordyca/support/depth0/controller_interactor_mapper.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class base_controller;
namespace depth1 {
class gp_dpo_controller;
class gp_mdpo_controller;
}
}
NS_START(support, depth1);
template<class T>
class robot_arena_interactor;

using gp_dpo_itype = robot_arena_interactor<controller::depth1::gp_dpo_controller>;
using gp_mdpo_itype = robot_arena_interactor<controller::depth1::gp_mdpo_controller>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class controller_interactor_mapper
 * @ingroup support depth1
 * @brief Helper class to map the typeid of a \ref controller::base_controller
 * to a \ref depth1::robot_arena_interactor to correctly handle robot-arena
 * interactions for that type.
 */

class controller_interactor_mapper : public depth0::controller_interactor_mapper {
 public:
  controller_interactor_mapper(controller::base_controller* const controller,
                               uint timestep)
      : depth0::controller_interactor_mapper(controller, timestep) {}

  /* depth1 */
  void operator()(gp_dpo_itype& interactor) const;
  void operator()(gp_mdpo_itype& interactor) const;
};

NS_END(depth1, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH1_CONTROLLER_INTERACTOR_MAPPER_HPP_ */
