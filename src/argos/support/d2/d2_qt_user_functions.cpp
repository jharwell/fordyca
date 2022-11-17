/**
 * \file d2_qt_user_functions.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_OVERLOADED_VIRTUAL()
#include "fordyca/argos/support/d2/d2_qt_user_functions.hpp"
RCPPSW_WARNING_DISABLE_POP()

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, d2);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

NS_END(support, fordyca, argos, d2);

using namespace fasd2; // NOLINT

RCPPSW_WARNING_DISABLE_PUSH()
RCPPSW_WARNING_DISABLE_MISSING_VAR_DECL()
RCPPSW_WARNING_DISABLE_MISSING_PROTOTYPE()
RCPPSW_WARNING_DISABLE_GLOBAL_CTOR()

REGISTER_QTOPENGL_USER_FUNCTIONS(d2_qt_user_functions, "d2_qt_user_functions");

RCPPSW_WARNING_DISABLE_POP()
