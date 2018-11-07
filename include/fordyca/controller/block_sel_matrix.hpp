/**
 * @file block_sel_matrix.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_FORDYCA_CONTROLLER_BLOCK_SEL_MATRIX_HPP_
#define INCLUDE_FORDYCA_CONTROLLER_BLOCK_SEL_MATRIX_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/math/vector2.h>
#include <boost/variant.hpp>
#include <map>
#include <string>
#include <vector>

#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);
namespace params {
struct block_sel_matrix_params;
}
NS_START(controller);
using block_sel_variant = boost::variant<double,
                                               argos::CVector2,
                                               std::vector<int>>;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class block_sel_matrix
 * @ingroup controller
 *
 * @brief A dictionary of information needed by robots using various utility
 * functions to calculate the best:
 *
 * - block (of whatever type)
 *
 * This class may be separated into those components in the future if it makes
 * sense. For now, it is cleaner to have all three uses be in the same class.
 */

class block_sel_matrix : public std::map<std::string, block_sel_variant> {
 public:
  static constexpr char kNestLoc[] = "nest_loc";
  static constexpr char kCubePriority[] = "cube_priority";
  static constexpr char kRampPriority[] = "ramp_priority";
  static constexpr char kSelExceptions[] = "sel_exceptions";

  explicit block_sel_matrix(
      const struct params::block_sel_matrix_params * params);

  void sel_exception_add(int id);
  void sel_exceptions_clear(void);
};

NS_END(controller, fordyca);

#endif /* INCLUDE_FORDYCA_CONTROLLER_BLOCK_SEL_MATRIX_HPP_ */
