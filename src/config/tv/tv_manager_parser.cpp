/**
 * @file tv_manager_parser.cpp
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
#include "fordyca/config/tv/tv_manager_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config, tv);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tv_manager_parser::parse(const ticpp::Element& node) {
  /* No temporal variance configured */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element tvnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  /* block temporal variance configured */
  if (nullptr != tvnode.FirstChild("blocks", false)) {
    ticpp::Element bnode = node_get(tvnode, "blocks");

    if (nullptr != bnode.FirstChild("manipulation_penalty", false)) {
      m_block_manip.parse(node_get(bnode, "manipulation_penalty"));
      m_config->block_manipulation_penalty =
          *m_block_manip
               .config_get<rct::config::xml::waveform_parser::config_type>();
    }
    if (nullptr != bnode.FirstChild("carry_throttle", false)) {
      m_block_carry.parse(node_get(bnode, "carry_throttle"));
      m_config->block_carry_throttle =
          *m_block_carry
               .config_get<rct::config::xml::waveform_parser::config_type>();
    }
  }

  /* cache temporal variance configured */
  if (nullptr != tvnode.FirstChild("caches", false)) {
    ticpp::Element cnode = node_get(tvnode, "caches");
    if (nullptr != cnode.FirstChild("usage_penalty", false)) {
      m_cache_usage.parse(node_get(cnode, "usage_penalty"));
      m_config->cache_usage_penalty =
          *m_cache_usage
               .config_get<rct::config::xml::waveform_parser::config_type>();
    }
  }
} /* parse() */

bool tv_manager_parser::validate(void) const {
  return m_block_manip.validate() && m_block_carry.validate() &&
         m_cache_usage.validate();
} /* validate() */

NS_END(tv, config, fordyca);
