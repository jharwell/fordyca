/**
 * @file depth0/controller_interactor_mapper.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_CONTROLLER_INTERACTOR_MAPPER_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_CONTROLLER_INTERACTOR_MAPPER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/nsalias.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca);

namespace controller {
class base_controller;
namespace depth0 {
class crw_controller;
class dpo_controller;
class mdpo_controller;
}
}
NS_START(support, depth0);
template<class T>
class robot_arena_interactor;

using crw_itype = robot_arena_interactor<controller::depth0::crw_controller>;
using dpo_itype = robot_arena_interactor<controller::depth0::dpo_controller>;
using mdpo_itype = robot_arena_interactor<controller::depth0::mdpo_controller>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * @class controller_interactor_mapper
 * @ingroup fordyca support depth0
 *
 * @brief Helper class to map the typeid of a \ref base_controller
 * to a \ref robot_arena_interactor to correctly handle robot-arena
 * interactions for that type.
 */
class controller_interactor_mapper {
 public:
  controller_interactor_mapper(controller::base_controller* const controller,
                          uint timestep)
      : m_timestep(timestep), m_controller(controller) {}

  /* depth0 */
  void operator()(crw_itype& interactor) const;
  void operator()(dpo_itype& interactor) const;
  void operator()(mdpo_itype& interactor) const;

 protected:
  uint timestep(void) const { return m_timestep; }
  controller::base_controller* controller(void) const { return m_controller; }

 private:
  /* clang-format off */
  uint                               m_timestep;
  controller::base_controller* const m_controller;
  /* clang-format on */
};

NS_END(depth0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_CONTROLLER_INTERACTOR_MAPPER_HPP_ */
