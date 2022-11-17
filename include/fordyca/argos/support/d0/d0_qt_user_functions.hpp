/**
 * \file d0_qt_user_functions.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/hal/robot.hpp"

#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d0);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d0_qt_user_functions
 * \ingroup support d0
 *
 * \brief Contains hooks for Qt to draw the robot's LOS if so configured.
 */
class d0_qt_user_functions : public ::argos::CQTOpenGLUserFunctions {
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

  d0_qt_user_functions(void);

  ~d0_qt_user_functions(void) override = default;

  void Draw(chal::robot& c_entity);
};

NS_END(d0, fordyca, argos, support);

