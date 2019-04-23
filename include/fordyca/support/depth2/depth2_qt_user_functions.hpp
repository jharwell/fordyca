/**
 * @file depth2_qt_user_functions.hpp
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
#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_QT_USER_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_QT_USER_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth1/depth1_qt_user_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth2);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class depth2_qt_user_functions
 * @ingroup fordyca support depth2
 *
 * @brief Contains hooks for Qt to draw the visualizations related to depth 2
 * task decomposition:
 *
 */
class depth2_qt_user_functions : public depth1::depth1_qt_user_functions {
 public:
  depth2_qt_user_functions(void) = default;
  ~depth2_qt_user_functions(void) override = default;
};

NS_END(depth2, support, fordyca);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH2_DEPTH2_QT_USER_FUNCTIONS_HPP_ */
