/**
 * @file collectible_task_diagnostics.hpp
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

#ifndef INCLUDE_FORDYCA_DIAGNOSTICS_DEPTH1_COLLECTIBLE_TASK_DIAGNOSTICS_HPP_
#define INCLUDE_FORDYCA_DIAGNOSTICS_DEPTH1_COLLECTIBLE_TASK_DIAGNOSTICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics, depth1);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class collectible_task_diagnostics {
 public:
  collectible_task_diagnostics(void) {}
  virtual ~collectible_task_diagnostics(void) {}

  virtual std::string task_name(void) const = 0;
};

NS_END(depth1, diagnostics, fordyca);

#endif /* INCLUDE_FORDYCA_DIAGNOSTICS_DEPTH1_COLLECTIBLE_TASK_DIAGNOSTICS_HPP_ */
