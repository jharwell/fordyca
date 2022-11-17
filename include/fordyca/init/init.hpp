/**
 * \file init.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/fordyca.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, init);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Initialize the FORDYCA library.
 *
 * Currently this just prints the version.
 */
void init(void) RCPPSW_LIB_INIT;

NS_END(init, fordyca);
