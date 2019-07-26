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
#include "fordyca/config/metrics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, config);

/*******************************************************************************
 * Class Constants
 ******************************************************************************/
const std::list<std::string> metrics_parser::xml_attr = {
    "fsm_movement",
    "fsm_collision_counts",
    "fsm_collision_locs",

    "block_acq_counts",
    "block_acq_locs",
    "block_acq_explore_locs",
    "block_acq_vector_locs",
    "block_transport",
    "block_manipulation",

    "cache_acq_counts",
    "cache_acq_locs",
    "cache_acq_explore_locs",
    "cache_acq_vector_locs",
    "cache_utilization",
    "cache_lifecycle",
    "cache_locations",

    "task_execution_generalist",
    "task_execution_collector",
    "task_execution_harvester",
    "task_execution_cache_starter",
    "task_execution_cache_finisher",
    "task_execution_cache_transferer",
    "task_execution_cache_collector",

    "task_tab_generalist",
    "task_tab_harvester",
    "task_tab_collector",

    "task_distribution",

    "swarm_dist_pos2D",
    "swarm_convergence",
    "temporal_variance",
    "perception_mdpo",
    "perception_dpo",
};

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void metrics_parser::parse(const ticpp::Element& node) {
  /* loop functions metrics not part of controller XML tree  */
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element mnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(mnode, m_config, output_dir);
    XML_PARSE_ATTR(mnode, m_config, collect_interval);

    for (auto& m : xml_attr) {
      if (mnode.HasAttribute(m)) {
        std::string tmp;
        node_attr_get(mnode, m, tmp);
        m_config->enabled[m] = tmp;
      }
    } /* for(&m..) */
  }
} /* parse() */

bool metrics_parser::validate(void) const {
  if (is_parsed()) {
    CHECK(m_config->collect_interval > 0);
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(config, fordyca);
