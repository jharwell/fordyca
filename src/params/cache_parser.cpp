/**
 * @file cache_parser.cpp
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
#include "fordyca/params/cache_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_parser::parse(argos::TConfigurationNode& node) {
  m_params.reset(new struct cache_params);

  argos::TConfigurationNode bnode = argos::GetNode(node, "caches");
  argos::GetNodeAttribute(bnode, "dimension", m_params->dimension);
  argos::GetNodeAttribute(bnode, "min_dist", m_params->min_dist);
  argos::GetNodeAttribute(bnode, "static_cache_size", m_params->static_cache_size);
  argos::GetNodeAttribute(bnode, "usage_penalty", m_params->usage_penalty);
  argos::GetNodeAttribute(bnode, "create_static_cache", m_params->create_static_cache);
  argos::GetNodeAttribute(bnode, "create_dynamic_caches", m_params->create_dynamic_caches);
} /* parse() */

void cache_parser::show(std::ostream& stream) {
  stream << "====================\nCache params\n====================\n";
  stream << "dimension=" << m_params->dimension << std::endl;
  stream << "min_dist=" << m_params->min_dist << std::endl;
  stream << "static_cache_size=" << m_params->static_cache_size << std::endl;
  stream << "usage_penalty=" << m_params->usage_penalty << std::endl;
  stream << "create_static_cache=" << m_params->create_static_cache << std::endl;
  stream << "create_dynamic_caches=" << m_params->create_dynamic_caches << std::endl;
} /* show() */

NS_END(params, fordyca);
