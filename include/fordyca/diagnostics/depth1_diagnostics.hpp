/**
 * @file depth1_diagnostics.hpp
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

#ifndef INCLUDE_FORDYCA_DIAGNOSTICS_DEPTH1_DIAGNOSTICS_HPP_
#define INCLUDE_FORDYCA_DIAGNOSTICS_DEPTH1_DIAGNOSTICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "fordyca/diagnostics/depth0_diagnostics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, diagnostics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class depth1_diagnostics: public depth0_diagnostics {
 public:
  depth1_diagnostics(void) {}
  virtual ~depth1_diagnostics(void) {}

  virtual bool is_searching_for_cache(void) const = 0;
  virtual bool is_transporting_to_cache(void) const = 0;
  virtual std::string task_name(void) const = 0;
};

NS_END(diagnostics, fordyca);

#endif /* INCLUDE_FORDYCA_DIAGNOSTICS_DEPTH1_DIAGNOSTICS_HPP_ */
