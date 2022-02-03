/**
 * \file d2_qt_user_functions.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#ifndef INCLUDE_FORDYCA_ARGOS_SUPPORT_D2_D2_QT_USER_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_ARGOS_SUPPORT_D2_D2_QT_USER_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/d1/d1_qt_user_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d2_qt_user_functions
 * \ingroup argos support d2
 *
 * \brief Contains hooks for Qt to draw the visualizations related to depth 2
 * task decomposition:
 *
 */
class d2_qt_user_functions : public d1::d1_qt_user_functions {
 public:
  d2_qt_user_functions(void) = default;
  ~d2_qt_user_functions(void) override = default;
};

NS_END(d2, support, argos, fordyca);

#endif /* INCLUDE_FORDYCA_ARGOS_SUPPORT_D2_D2_QT_USER_FUNCTIONS_HPP_ */
