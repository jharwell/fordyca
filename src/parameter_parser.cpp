/**
 * @file parameter_parser.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fordyca/parameter_parser.hpp"
#include <algorithm>
#include "rcsw/common/fpc.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
status_t parameter_parser::add_category(const std::string& name, const base_param_parser& parser) {
  FPC_CHECK(ERROR, m_parsers.find(name) != m_parsers.end());

  m_parsers.insert(std::pair<std::string, base_param_parser>(name, parser));
  return OK;
} /* parameter_parser:add_category() */

status_t parameter_parser::parse(argos::TConfigurationNode& node) {
  std::for_each(m_parsers.begin(), m_parsers.end(), [&](std::pair<const std::string, base_param_parser>& pair) {
      pair.second.parse(node);
    });
  return OK;
} /* parameter_parser:parse() */

NS_END(fordyca);
