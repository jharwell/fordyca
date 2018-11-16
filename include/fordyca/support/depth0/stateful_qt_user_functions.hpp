/**
 * @file stateful_qt_user_functions.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_QT_USER_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_QT_USER_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/support/depth0/crw_qt_user_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * @class stateful_qt_user_functions
 * @ingroup support depth0
 *
 * @brief Contains hooks for Qt to draw the robot's LOS if so configured.
 */
class stateful_qt_user_functions : public crw_qt_user_functions {
 public:
  stateful_qt_user_functions(void);

  ~stateful_qt_user_functions(void) override = default;

  void Draw(argos::CFootBotEntity& c_entity) override;
};

NS_END(depth0, fordyca, support);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_STATEFUL_QT_USER_FUNCTIONS_HPP_ */
