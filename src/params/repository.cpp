/**
 * @file repository.cpp
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
#include <algorithm>
#include "rcsw/common/fpc.h"
#include "fordyca/params/repository.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void repository::parse_all(argos::TConfigurationNode& node) {
  std::for_each(m_parsers.begin(),
                m_parsers.end(),
                [&](std::pair<const std::string, base_param_parser*>& pair) {
      pair.second->parse(node);
    });
} /* parse_all() */

void repository::show_all(std::ostream& stream) {
  std::for_each(m_parsers.begin(),
                m_parsers.end(),
                [&](std::pair<const std::string, base_param_parser*>& pair) {
      pair.second->show(stream);
    });
} /* show_all() */

NS_END(params, fordyca);
