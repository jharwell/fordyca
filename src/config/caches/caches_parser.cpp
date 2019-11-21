/**
 * \file caches_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "fordyca/config/caches/caches_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, caches);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void caches_parser::parse(const ticpp::Element& node) {
  /* caches not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_static.parse(cnode);
  if (m_static.is_parsed()) {
    m_config->static_ = *m_static.config_get<static_cache_parser::config_type>();
  }

  m_dynamic.parse(cnode);
  if (m_dynamic.is_parsed()) {
    m_config->dynamic =
        *m_dynamic.config_get<dynamic_cache_parser::config_type>();
  }

  XML_PARSE_ATTR(cnode, m_config, dimension);
} /* parse() */

bool caches_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }

  RCSW_CHECK(m_config->dimension > 0.0);
  RCSW_CHECK(m_dynamic.validate());
  RCSW_CHECK(m_static.validate());
  return true;

error:
  return false;
} /* validate() */

NS_END(caches, config, fordyca);
