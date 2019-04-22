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
#include "fordyca/params/metrics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

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
    XML_PARSE_ATTR(mnode, m_params, fsm_collision_fname);
    XML_PARSE_ATTR(mnode, m_params, fsm_movement_fname);

    XML_PARSE_ATTR(mnode, m_params, block_transport_fname);
    XML_PARSE_ATTR(mnode, m_params, block_acquisition_counts_fname);
    XML_PARSE_ATTR(mnode, m_params, block_acquisition_locs_fname);
    XML_PARSE_ATTR(mnode, m_params, block_manipulation_fname);

    XML_PARSE_ATTR(mnode, m_params, cache_acquisition_counts_fname);
    XML_PARSE_ATTR(mnode, m_params, cache_acquisition_locs_fname);
    XML_PARSE_ATTR(mnode, m_params, cache_utilization_fname);
    XML_PARSE_ATTR(mnode, m_params, cache_lifecycle_fname);
    XML_PARSE_ATTR(mnode, m_params, cache_locations_fname);

    XML_PARSE_ATTR(mnode, m_params, task_execution_generalist_fname);
    XML_PARSE_ATTR(mnode, m_params, task_execution_collector_fname);
    XML_PARSE_ATTR(mnode, m_params, task_execution_harvester_fname);
    XML_PARSE_ATTR(mnode, m_params, task_execution_cache_starter_fname);
    XML_PARSE_ATTR(mnode, m_params, task_execution_cache_finisher_fname);
    XML_PARSE_ATTR(mnode, m_params, task_execution_cache_transferer_fname);
    XML_PARSE_ATTR(mnode, m_params, task_execution_cache_collector_fname);

    XML_PARSE_ATTR(mnode, m_params, task_tab_generalist_fname);
    XML_PARSE_ATTR(mnode, m_params, task_tab_collector_fname);
    XML_PARSE_ATTR(mnode, m_params, task_tab_harvester_fname);

    XML_PARSE_ATTR(mnode, m_params, task_distribution_fname);

    XML_PARSE_ATTR(mnode, m_params, output_dir);

    XML_PARSE_ATTR(mnode, m_params, perception_mdpo_fname);
    XML_PARSE_ATTR(mnode, m_params, perception_dpo_fname);

    XML_PARSE_ATTR(mnode, m_params, arena_robot_locs_fname);
    XML_PARSE_ATTR(mnode, m_params, swarm_convergence_fname);
    XML_PARSE_ATTR(mnode, m_params, loop_temporal_variance_fname);
    XML_PARSE_ATTR(mnode, m_params, collect_interval);
    m_parsed = true;
  }
} /* parse() */

__rcsw_pure bool metrics_parser::validate(void) const {
  if (m_parsed) {
    CHECK(m_params->collect_interval > 0);
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(params, fordyca);
