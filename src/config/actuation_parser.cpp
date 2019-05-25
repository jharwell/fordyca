/**
 * @file actuation_parser.cpp
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
#include "fordyca/config/actuation_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char actuation_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuation_parser::parse(const ticpp::Element& node) {
  ticpp::Element anode = node_get(node, kXMLRoot);
  m_differential_drive.parse(anode);
  m_steering.parse(anode);

  m_config =
      std::make_shared<std::remove_reference<decltype(*m_config)>::type>();
  m_config->differential_drive = *m_differential_drive.config_get();
  m_config->steering = *m_steering.config_get();
} /* parse() */

__rcsw_pure bool actuation_parser::validate(void) const {
  CHECK(m_differential_drive.validate());
  CHECK(m_steering.validate());
  return true;

error:
  return false;
} /* validate() */

NS_END(config, fordyca);