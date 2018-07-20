/**
 * @file task_repository.cpp
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
#include "fordyca/params/depth1/param_repository.hpp"
#include "fordyca/params/depth1/exec_estimates_parser.hpp"
#include "rcppsw/er/server.hpp"
#include "rcppsw/control/waveform_xml_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace ct = rcppsw::control;
NS_START(fordyca, params, depth1);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
param_repository::param_repository(std::shared_ptr<rcppsw::er::server>& server)
    : stateful_param_repository(server) {
  register_parser<exec_estimates_parser, exec_estimates_params>(
      exec_estimates_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);
  register_parser<ct::waveform_xml_parser>(std::string("cache_usage_") +
                                           ct::waveform_xml_parser::kXMLRoot,
                                           ct::waveform_xml_parser::kHeader1);
}

NS_END(depth1, params, fordyca);
