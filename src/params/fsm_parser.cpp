/**
 * @file fsm_parser.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/core/utility/configuration/argos_configuration.h>

#include "fordyca/params/fsm_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"
#include "rcsw/common/common.h"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char fsm_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void fsm_parser::parse(const ticpp::Element& node) {
  ticpp::Element fnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  XML_PARSE_PARAM(fnode, m_params.times, unsuccessful_explore_dir_change);
  XML_PARSE_PARAM(fnode, m_params.times, frequent_collision_thresh);
  XML_PARSE_PARAM(fnode, m_params.times, unsuccessful_explore_dir_change);

  rcppsw::utils::line_parser parser(' ');
  std::vector<std::string> res;
  res = parser.parse(fnode.GetAttribute("nest"));
  m_params.nest_center.Set(std::atof(res[0].c_str()), std::atof(res[1].c_str()));
} /* parse() */

void fsm_parser::show(std::ostream& stream) const {
  stream << build_header() << std::endl
         << XML_PARAM_STR(m_params.times, unsuccessful_explore_dir_change)
         << std::endl
         << XML_PARAM_STR(m_params.times, frequent_collision_thresh)
         << std::endl
         << XML_PARAM_STR(m_params.times, unsuccessful_explore_dir_change)
         << std::endl
         << XML_PARAM_STR(m_params.speed_throttling, block_carry) << std::endl
         << m_params.nest_center << std::endl;
} /* show() */

__pure bool fsm_parser::validate(void) const {
  return (m_params.nest_center.GetX() > 0) &&
         (m_params.nest_center.GetY() > 0);
} /* validate() */

NS_END(params, fordyca);
