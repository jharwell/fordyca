/**
 * @file robot_interactor_adaptor.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_ROBOT_INTERACTOR_ADAPTOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_ROBOT_INTERACTOR_ADAPTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/controller/controller_fwd.hpp"
#include "fordyca/nsalias.hpp"
#include "rcppsw/ds/type_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class robot_interactor_adaptor
 * @ingroup fordyca support
 *
 * @brief Wrapping functor to perform robot-arena interactions each
 * timestep. Needed for use with boost::static_visitor.
 */
template <template <typename ControllerType> class InteractorType,
          class RetType = void>
class robot_interactor_adaptor {
 public:
  robot_interactor_adaptor(controller::base_controller* const controller,
                           uint timestep)
      : m_timestep(timestep), m_controller(controller) {}

  template <typename T>
  RetType operator()(InteractorType<T>& interactor) const {
    auto controller =
        dynamic_cast<typename InteractorType<T>::controller_type*>(m_controller);
    return interactor(*controller, m_timestep);
  }

 private:
  /* clang-format off */
  uint                               m_timestep;
  controller::base_controller* const m_controller;
  /* clang-format on */
};

NS_END(support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_ROBOT_INTERACTOR_ADAPTOR_HPP_ */
