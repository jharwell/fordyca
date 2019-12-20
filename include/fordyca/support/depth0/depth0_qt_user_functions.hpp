/**
 * \file depth0_qt_user_functions.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#ifndef INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_QT_USER_FUNCTIONS_HPP_
#define INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_QT_USER_FUNCTIONS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, support, depth0);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class depth0_qt_user_functions
 * \ingroup support depth0
 *
 * \brief Contains hooks for Qt to draw the robot's LOS if so configured.
 */
class depth0_qt_user_functions : public argos::CQTOpenGLUserFunctions {
 public:
  /**
   * \brief How far above the center of the robot to draw the carried block (if
   * the robot is carrying a block)
   */
  static constexpr double kBLOCK_VIS_OFFSET = 0.3;

  /**
   * \brief How far above the center of the robot to draw text (robot id, task,
   * etc.)
   */
  static constexpr double kTEXT_VIS_OFFSET = 0.5;

  depth0_qt_user_functions(void);

  ~depth0_qt_user_functions(void) override = default;

  void Draw(argos::CFootBotEntity& c_entity);
};

NS_END(depth0, fordyca, support);

#endif /* INCLUDE_FORDYCA_SUPPORT_DEPTH0_DEPTH0_QT_USER_FUNCTIONS_HPP_ */
