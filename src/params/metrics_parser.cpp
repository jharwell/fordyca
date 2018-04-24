/**
 * @file metrics_parser.cpp
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

#include "fordyca/params/metrics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char metrics_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void metrics_parser::parse(const ticpp::Element& node) {
  ticpp::Element mnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  XML_PARSE_PARAM(mnode, m_params, output_dir);
  XML_PARSE_PARAM(mnode, m_params, block_acquisition_fname);
  XML_PARSE_PARAM(mnode, m_params, cache_acquisition_fname);
  XML_PARSE_PARAM(mnode, m_params, distance_fname);
  XML_PARSE_PARAM(mnode, m_params, block_transport_fname);
  XML_PARSE_PARAM(mnode, m_params, block_fname);
  XML_PARSE_PARAM(mnode, m_params, task_execution_fname);
  XML_PARSE_PARAM(mnode, m_params, task_management_fname);
  XML_PARSE_PARAM(mnode, m_params, task_management_fname);
  XML_PARSE_PARAM(mnode, m_params, cache_fname);
  XML_PARSE_PARAM(mnode, m_params, collect_interval);

  m_parsed = true;
} /* parse() */

void metrics_parser::show(std::ostream& stream) const {
  stream << build_header();
  if (!m_parsed) {
    stream << "<<  Not Parsed >>" << std::endl
    << build_footer();
    return;
  }
  stream << XML_PARAM_STR(m_params, output_dir) << std::endl
         << XML_PARAM_STR(m_params, block_acquisition_fname) << std::endl
         << XML_PARAM_STR(m_params, distance_fname) << std::endl
         << XML_PARAM_STR(m_params, block_transport_fname) << std::endl
         << XML_PARAM_STR(m_params, cache_acquisition_fname) << std::endl
         << XML_PARAM_STR(m_params, block_fname) << std::endl
         << XML_PARAM_STR(m_params, task_execution_fname) << std::endl
         << XML_PARAM_STR(m_params, task_management_fname) << std::endl
         << XML_PARAM_STR(m_params, task_management_fname) << std::endl
         << XML_PARAM_STR(m_params, cache_fname) << std::endl
         << XML_PARAM_STR(m_params, collect_interval) << std::endl
         << build_footer();
} /* show() */

__pure bool metrics_parser::validate(void) const {
  if (m_parsed) {
    return (0 != m_params.collect_interval);
  }
  return true;
} /* validate() */

NS_END(params, fordyca);
