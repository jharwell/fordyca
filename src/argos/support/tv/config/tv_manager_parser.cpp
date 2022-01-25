/**
 * \file tv_manager_parser.cpp
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
#include "fordyca/argos/support/tv/config/tv_manager_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, argos, support, tv, config);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tv_manager_parser::parse(const ticpp::Element& node) {
  /*
   * This needs to be non-NULL in ALL situations, because the environmental
   * dynamics part of temporal variance must ALWAYS be present (even if it is
   * not used). Using whether the config is NULL or not and creating a
   * std::unique_ptr if it is results in use-after-free. See FORDYCA#621.
   */
  m_config = std::make_unique<config_type>();

  /* No temporal variance configured */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  ticpp::Element tvnode = node_get(node, kXMLRoot);

  m_envd.parse(tvnode);
  m_popd.parse(tvnode);

  if (m_envd.is_parsed()) {
    m_config->env_dynamics =
        *m_envd.config_get<env_dynamics_parser::config_type>();
  }

  if (m_popd.is_parsed()) {
    m_config->population_dynamics = *m_popd.config_get<
        ctv::config::xml::population_dynamics_parser::config_type>();
  }
} /* parse() */

bool tv_manager_parser::validate(void) const {
  ER_CHECK(m_envd.validate(), "Environmental dynamics validation failed");
  ER_CHECK(m_popd.validate(), "Population dynamics validation failed");

  return true;

error:
  return false;
} /* validate() */

NS_END(config, tv, support, argos, fordyca);
