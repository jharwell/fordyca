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
#include "fordyca/params/output_parser.hpp"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char output_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void output_parser::parse(const ticpp::Element& node) {
  ticpp::Element onode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  std::vector<std::string> res, res2;

  /* only present for loop functions */
  if (nullptr != onode.FirstChild(metrics_parser::kXMLRoot, false)) {
    m_metrics_parser.parse(onode);
    m_params.metrics = *m_metrics_parser.parse_results();
  }

  ticpp::Element snode = argos::GetNode(onode, "sim");
  XML_PARSE_PARAM(snode, m_params, output_root);
  XML_PARSE_PARAM(snode, m_params, output_dir);

  /* only present for loop functions */
  if (snode.HasAttribute("log_fname")) {
    XML_PARSE_PARAM(snode, m_params, log_fname);
  }
} /* parse() */

void output_parser::show(std::ostream& stream) const {
  stream << build_header() << m_metrics_parser
         << XML_PARAM_STR(m_params, output_root) << std::endl
         << XML_PARAM_STR(m_params, output_dir) << std::endl
         << XML_PARAM_STR(m_params, log_fname) << std::endl
         << build_footer();
} /* show() */

__pure bool output_parser::validate(void) const {
  return m_metrics_parser.validate();
} /* validate() */

NS_END(params, fordyca);
