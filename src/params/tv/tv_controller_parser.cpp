/**
 * @file tv_controller_parser.cpp
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
#include "fordyca/params/tv/tv_controller_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, tv);

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
constexpr char tv_controller_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tv_controller_parser::parse(const ticpp::Element& node) {
  /* No temporal variance configured */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element tvnode = get_node(const_cast<ticpp::Element&>(node), kXMLRoot);
  m_params =
      std::make_shared<std::remove_reference<decltype(*m_params)>::type>();

  /* block temporal variance configured */
  if (nullptr != tvnode.FirstChild("blocks", false)) {
    ticpp::Element bnode =
        get_node(const_cast<ticpp::Element&>(tvnode), "blocks");

    if (nullptr != bnode.FirstChild("manipulation_penalty", false)) {
      m_block_manip.parse(
          get_node(const_cast<ticpp::Element&>(bnode), "manipulation_penalty"));
      m_params->block_manipulation_penalty = *m_block_manip.parse_results();
    }
    if (nullptr != bnode.FirstChild("carry_throttle", false)) {
      m_block_carry.parse(
          get_node(const_cast<ticpp::Element&>(bnode), "carry_throttle"));
      m_params->block_carry_throttle = *m_block_carry.parse_results();
    }
  }

  /* cache temporal variance configured */
  if (nullptr != tvnode.FirstChild("caches", false)) {
    ticpp::Element cnode =
        get_node(const_cast<ticpp::Element&>(tvnode), "caches");
    if (nullptr != cnode.FirstChild("usage_penalty", false)) {
      m_cache_usage.parse(get_node(const_cast<ticpp::Element&>(cnode),
                                   "usage_penalty"));
      m_params->cache_usage_penalty = *m_cache_usage.parse_results();
    }
  }
} /* parse() */

__rcsw_pure bool tv_controller_parser::validate(void) const {
  return m_block_manip.validate() && m_block_carry.validate() &&
         m_cache_usage.validate();
} /* validate() */

NS_END(tv, params, fordyca);
