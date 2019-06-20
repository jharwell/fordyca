/**
 * @file output_parser.cpp
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
#include "fordyca/config/output_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char output_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void output_parser::parse(const ticpp::Element& node) {
  ticpp::Element onode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  std::vector<std::string> res, res2;

  /* only present for loop functions */
  if (nullptr != onode.FirstChild(metrics_parser::kXMLRoot, false)) {
    m_metrics_parser.parse(onode);
    m_config->metrics =
        *m_metrics_parser.config_get<metrics_parser::config_type>();
  }

  ticpp::Element snode = node_get(onode, "sim");
  XML_PARSE_ATTR(snode, m_config, output_root);
  XML_PARSE_ATTR(snode, m_config, output_dir);
} /* parse() */

__rcsw_pure bool output_parser::validate(void) const {
  return m_metrics_parser.validate();
} /* validate() */

NS_END(config, fordyca);
