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

#include "rcppsw/common/licensing.hpp"

#include <iostream>

#include "fordyca/version.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(fordyca, init);

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
void init(void) {
  std::cout << "Loaded FORDYCA, " << kVERSION << ": ";
  std::cout << RCPPSW_LICENSE(LGPLV3, FORDYCA, 2022, John Harwell) << std::endl;
} /* init() */

NS_END(init, fordyca);
