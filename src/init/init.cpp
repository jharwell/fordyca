/**
 * \file init.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
  std::cout << RCPPSW_LICENSE(GPLV3, FORDYCA, 2022, John Harwell) << std::endl;
} /* init() */

NS_END(init, fordyca);
