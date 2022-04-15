/**
 * \file env_dynamics_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "fordyca/argos/support/tv/config/env_dynamics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv, config);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
env_dynamics_parser::env_dynamics_parser(void)
    : ER_CLIENT_INIT("fordyca.argos support.config.env_dynamics_parser") {
  m_motion.xml_root("motion_throttle");
  m_block_manip.xml_root("manipulation_penalty");
  m_block_carry.xml_root("carry_throttle");
  m_cache_usage.xml_root("usage_penalty");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_dynamics_parser::parse(const ticpp::Element& node) {
  using penalty_config_type =
      ctv::config::xml::temporal_penalty_parser::config_type;

  /* No environmental dynamics configured */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element tvnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  /* motion dynamics configured */
  if (nullptr != tvnode.FirstChild(m_motion.xml_root(), false)) {
    m_motion.parse(tvnode);
    const auto* config = m_motion.config_get<penalty_config_type>();
    m_config->rda.motion_throttle = config->waveform;
  }
  /* block dynamics configured */
  if (nullptr != tvnode.FirstChild("blocks", false)) {
    ticpp::Element bnode = node_get(tvnode, "blocks");

    m_block_manip.parse(bnode);
    if (m_block_manip.is_parsed()) {
      const auto* config = m_block_manip.config_get<penalty_config_type>();
      m_config->block_manip_penalty = *config;
    }

    m_block_carry.parse(bnode);
    if (m_block_carry.is_parsed()) {
      const auto* config = m_block_carry.config_get<penalty_config_type>();
      m_config->rda.block_carry_throttle = config->waveform;
    }
  }

  /* cache dynamics configured */
  if (nullptr != tvnode.FirstChild("caches", false)) {
    ticpp::Element cnode = node_get(tvnode, "caches");
    m_cache_usage.parse(cnode);
    if (m_cache_usage.is_parsed()) {
      const auto* config = m_cache_usage.config_get<penalty_config_type>();
      m_config->cache_usage_penalty = *config;
    }
  }
} /* parse() */

bool env_dynamics_parser::validate(void) const {
  ER_CHECK(m_block_manip.validate(),
           "Block manipulation dynamics validation faild");
  ER_CHECK(m_block_carry.validate(), "Block carry validation failed");
  ER_CHECK(m_cache_usage.validate(), "Cache usage validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, tv, support, argos, fordyca);
