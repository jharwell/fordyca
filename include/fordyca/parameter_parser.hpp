/**
 * @file parameter_parser.hpp
 *
 * Handles parsing of all XML parameters at runtime.
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

#ifndef INCLUDE_FORDYCA_PARAMETER_PARSER_HPP_
#define INCLUDE_FORDYCA_PARAMETER_PARSER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcsw/common/common.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace fordyca {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

class parameter_parser {
 public:
  /* constructors */
  parameter_parser();

  /* member functions */
  template<typename T>
  status_t add_category(const std::string& category);
  status_t parse_categories(void);

 private:
  /* member functions */
  /* data members */
};

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/*******************************************************************************
 * Operater Definitions
 ******************************************************************************/

} /* namespace fordyca */

#endif /* INCLUDE_FORDYCA_PARAMETER_PARSER_HPP_ */
