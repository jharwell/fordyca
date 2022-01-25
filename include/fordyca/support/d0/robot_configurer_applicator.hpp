/**
 * \file d0/robot_configurer_applicator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#ifndef INCLUDE_FORDYCA_SUPPORT_D0_ROBOT_CONFIGURER_APPLICATOR_HPP_
#define INCLUDE_FORDYCA_SUPPORT_D0_ROBOT_CONFIGURER_APPLICATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"
#include "fordyca/controller/controller_fwd.hpp"
#include "rcppsw/ds/type_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, support, d0);
template<class T>
class robot_configurer;

using configurer_map_type = rds::type_map<
  rmpl::typelist_wrap_apply<controller::d0::typelist,
                            robot_configurer>::type>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_configurer_applicator
 * \ingroup support d0
 *
 * \brief Wrapping functor to perform robot controller configuration during
 * initialization.
 */
class robot_configurer_applicator {
 public:
  explicit robot_configurer_applicator(controller::foraging_controller* const c)
      : m_controller(c) {}

  template<typename T>
  void operator()(robot_configurer<T>& configurer) const {
    auto *cast = dynamic_cast<typename support::d0::robot_configurer<T>::controller_type*>(m_controller);
    configurer(cast);
  }

 private:
  /* clang-format off */
  controller::foraging_controller* const m_controller;
  /* clang-format on */
};

NS_END(d0, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_D0_ROBOT_CONFIGURER_APPLICATOR_HPP_ */
