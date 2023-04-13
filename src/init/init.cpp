/**
 * \file init.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/init/init.hpp"

#include "rcppsw/version/version.hpp"

#include <iostream>

#include "fordyca/version/version.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, init);

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
void init(void) {
  std::cout << "Loaded FOraging Robots use DYnamic CAches (FORDYCA): ";
  std::cout << rversion::meta_info_to_str(&version::kVersion);
} /* init() */

NS_END(init, fordyca);
