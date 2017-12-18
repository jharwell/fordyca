/**
 * @file output_parser.cpp
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
#include "fordyca/params/output_parser.hpp"
#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void output_parser::parse(argos::TConfigurationNode &node) {
  m_params.reset(new struct output_params);
  std::vector<std::string> res, res2;
  argos::TConfigurationNode onode = argos::GetNode(node, "output");

  if (onode.FirstChild("metrics", false)) {
    m_metrics_parser.parse(argos::GetNode(onode, "metrics"));
    m_params->metrics = *m_metrics_parser.get_results();
  }

  if (onode.FirstChild("sim", false)) {
    argos::TConfigurationNode snode = argos::GetNode(onode, "sim");
    argos::GetNodeAttribute(snode, "output_root", m_params->output_root);
    argos::GetNodeAttribute(snode, "output_dir", m_params->output_dir);
    argos::GetNodeAttribute(snode, "sim_log_fname", m_params->sim_log_fname);
  } else if (onode.FirstChild("robot", false)) {
    argos::TConfigurationNode rnode = argos::GetNode(onode, "robot");
    argos::GetNodeAttribute(rnode, "output_root", m_params->output_root);
    argos::GetNodeAttribute(rnode, "output_dir", m_params->output_dir);
  }
} /* parse() */

void output_parser::show(std::ostream &stream) {
  stream << "====================\nOutput params\n====================\n";
  m_metrics_parser.show(stream);
  stream << "output_root=" << m_params->output_root << std::endl;
  stream << "output_dir=" << m_params->output_dir << std::endl;
  stream << "sim_log_fname=" << m_params->sim_log_fname << std::endl;
} /* show() */

NS_END(params, fordyca);
