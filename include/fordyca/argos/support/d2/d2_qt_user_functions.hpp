/**
 * \file d2_qt_user_functions.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */
#pragma once

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

