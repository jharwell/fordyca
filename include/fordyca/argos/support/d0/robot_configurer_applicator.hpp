/**
 * \file d0/robot_configurer_applicator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/type_map.hpp"

#include "fordyca/fordyca.hpp"
#include "fordyca/controller/controller_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, argos, support, d0);

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
 * \ingroup argos support d0
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

NS_END(d0, support, argos, fordyca);

