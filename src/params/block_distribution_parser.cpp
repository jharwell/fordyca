/**
 * @file block_distribution_parser.cpp
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

#include "fordyca/params/block_distribution_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char block_distribution_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_distribution_parser::parse(const ticpp::Element& node) {
  ticpp::Element bnode =
      argos::GetNode(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();
  XML_PARSE_PARAM(bnode, m_params, n_blocks);
  XML_PARSE_PARAM(bnode, m_params, arena_resolution);
  XML_PARSE_PARAM(bnode, m_params, dist_type);
  XML_PARSE_PARAM(bnode, m_params, pwr_min);
  XML_PARSE_PARAM(bnode, m_params, pwr_max);
  XML_PARSE_PARAM(bnode, m_params, n_clusters);
} /* parse() */

void block_distribution_parser::show(std::ostream& stream) const {
  stream << build_header() << XML_PARAM_STR(m_params, dist_type) << std::endl
         << XML_PARAM_STR(m_params, n_blocks) << std::endl
         << XML_PARAM_STR(m_params, arena_resolution) << std::endl
         << XML_PARAM_STR(m_params, dist_type) << std::endl
         << XML_PARAM_STR(m_params, pwr_min) << std::endl
         << XML_PARAM_STR(m_params, pwr_max) << std::endl
         << XML_PARAM_STR(m_params, n_clusters) << std::endl
         << build_footer();
} /* show() */

bool block_distribution_parser::validate(void) const {
  CHECK(m_params->n_blocks > 0);
  CHECK("" != m_params->dist_type);
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
