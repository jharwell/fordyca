/**
 * @file depth2/controller_interactor_mapper.hpp
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

#ifndef INCLUDE_FORDYCA_SUPPORT_CONTROLLER_INTERACTOR_MAPPER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_CONTROLLER_INTERACTOR_MAPPER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"
#include "fordyca/support/depth1/controller_interactor_mapper.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class base_controller;
namespace depth2 {
class grp_dpo_controller;
class grp_mdpo_controller;
}
}
NS_START(support, depth2);

template<typename T>
class robot_arena_interactor;

using grp_dpo_itype = depth2::robot_arena_interactor<controller::depth2::grp_dpo_controller>;
using grp_mdpo_itype = depth2::robot_arena_interactor<controller::depth2::grp_mdpo_controller>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class controller_interactor_mapper
 * @ingroup fordyca support depth2
 *
 * @brief Helper class to map the typeid of a \ref base_controller to a \ref
 * depth2::robot_arena_interactor to correctly handle robot-arena interactions
 * for that type.
 */

class controller_interactor_mapper : public depth1::controller_interactor_mapper {
 public:
  controller_interactor_mapper(controller::base_controller* const controller,
                               uint timestep)
      : depth1::controller_interactor_mapper(controller, timestep) {}

  /* depth2 */
  bool operator()(grp_dpo_itype& interactor) const;
  bool operator()(grp_mdpo_itype& interactor) const;
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_ROBOT_INTERACTOR_MAPPER_HPP_ */
