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
 * Global Variables
 ******************************************************************************/
constexpr char metrics_parser::kXMLRoot[];

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void metrics_parser::parse(const ticpp::Element& node) {
  ticpp::Element mnode = node_get(node, kXMLRoot);

  /* loop functions metrics not part of controller XML tree  */
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    XML_PARSE_ATTR(mnode, m_config, fsm_collision_counts_fname);
    XML_PARSE_ATTR(mnode, m_config, fsm_collision_locs_fname);
    XML_PARSE_ATTR(mnode, m_config, fsm_movement_fname);

    XML_PARSE_ATTR(mnode, m_config, block_transport_fname);
    XML_PARSE_ATTR(mnode, m_config, block_acq_locs_fname);
    XML_PARSE_ATTR(mnode, m_config, block_acq_explore_locs_fname);
    XML_PARSE_ATTR(mnode, m_config, block_acq_vector_locs_fname);
    XML_PARSE_ATTR(mnode, m_config, block_acq_counts_fname);
    XML_PARSE_ATTR(mnode, m_config, block_manipulation_fname);

    XML_PARSE_ATTR(mnode, m_config, cache_acq_locs_fname);
    XML_PARSE_ATTR(mnode, m_config, cache_acq_explore_locs_fname);
    XML_PARSE_ATTR(mnode, m_config, cache_acq_vector_locs_fname);
    XML_PARSE_ATTR(mnode, m_config, cache_acq_counts_fname);
    XML_PARSE_ATTR(mnode, m_config, cache_utilization_fname);
    XML_PARSE_ATTR(mnode, m_config, cache_lifecycle_fname);
    XML_PARSE_ATTR(mnode, m_config, cache_locations_fname);

    XML_PARSE_ATTR(mnode, m_config, task_execution_generalist_fname);
    XML_PARSE_ATTR(mnode, m_config, task_execution_collector_fname);
    XML_PARSE_ATTR(mnode, m_config, task_execution_harvester_fname);
    XML_PARSE_ATTR(mnode, m_config, task_execution_cache_starter_fname);
    XML_PARSE_ATTR(mnode, m_config, task_execution_cache_finisher_fname);
    XML_PARSE_ATTR(mnode, m_config, task_execution_cache_transferer_fname);
    XML_PARSE_ATTR(mnode, m_config, task_execution_cache_collector_fname);

    XML_PARSE_ATTR(mnode, m_config, task_tab_generalist_fname);
    XML_PARSE_ATTR(mnode, m_config, task_tab_collector_fname);
    XML_PARSE_ATTR(mnode, m_config, task_tab_harvester_fname);

    XML_PARSE_ATTR(mnode, m_config, task_distribution_fname);

    XML_PARSE_ATTR(mnode, m_config, output_dir);

    XML_PARSE_ATTR(mnode, m_config, perception_mdpo_fname);
    XML_PARSE_ATTR(mnode, m_config, perception_dpo_fname);

    XML_PARSE_ATTR(mnode, m_config, swarm_dist_pos2D_fname);
    XML_PARSE_ATTR(mnode, m_config, swarm_convergence_fname);
    XML_PARSE_ATTR(mnode, m_config, temporal_variance_fname);
    XML_PARSE_ATTR(mnode, m_config, collect_interval);
    m_parsed = true;
  }
} /* parse() */

__rcsw_pure bool metrics_parser::validate(void) const {
  if (m_parsed) {
    CHECK(m_config->collect_interval > 0);
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(config, fordyca);
