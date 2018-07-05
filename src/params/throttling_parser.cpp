/**
 * @file throttling_parser.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#include "fordyca/params/throttling_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char throttling_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void throttling_parser::parse(const ticpp::Element& node) {
    if (nullptr != node.FirstChild(kXMLRoot, false)) {
      ticpp::Element tnode =
          argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
      m_params =
          std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
      XML_PARSE_PARAM(tnode, m_params, block_carry);
      m_parsed = true;
    }
} /* parse() */

void throttling_parser::show(std::ostream& stream) const {
  if (!m_parsed) {
    stream << build_header()
           << "<< Not Parsed >>"
           << std::endl
           << build_footer();
    return;
  }

  stream << build_header() << XML_PARAM_STR(m_params, block_carry) << std::endl
         << build_footer();
} /* show() */

__rcsw_pure bool throttling_parser::validate(void) const {
  if (m_parsed) {
    CHECK(m_params->block_carry >= 0.0);
    CHECK(m_params->block_carry <= 1.0);
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
