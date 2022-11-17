/**
 * \file d1_qt_user_functions.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/argos/support/d0/d0_qt_user_functions.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d1);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class d1_qt_user_functions
 * \ingroup argos support d1
 *
 * \brief Contains hooks for Qt to draw the visualizations related to depth 1
 * task decomposition:
 *
 * - Task name
 */
class d1_qt_user_functions : public d0::d0_qt_user_functions {
 public:
  d1_qt_user_functions(void);
  ~d1_qt_user_functions(void) override = default;

  void Draw(chal::robot& c_entity);
};

NS_END(d1, support, argos, fordyca);

