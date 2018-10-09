/**
 * @file loop_function_repository.cpp
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
#include "fordyca/params/loop_function_repository.hpp"
#include "fordyca/params/arena/arena_map_parser.hpp"
#include "fordyca/params/battery_parser.hpp"
#include "fordyca/params/oracle_parser.hpp"
#include "fordyca/params/output_parser.hpp"
#include "fordyca/params/visualization_parser.hpp"
#include "rcppsw/control/waveform_xml_parser.hpp"
#include "fordyca/params/oracle_parser.hpp"
#include "fordyca/params/battery_parser.hpp"
#include "fordyca/params/caches/caches_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params);
namespace ct = rcppsw::control;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
loop_function_repository::loop_function_repository(void) {
  register_parser<output_parser, output_params>(
      output_parser::kXMLRoot, rcppsw::params::xml_param_parser::kHeader1);
  register_parser<arena::arena_map_parser, arena::arena_map_params>(
      arena::arena_map_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);
  register_parser<visualization_parser, visualization_params>(
      visualization_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);
  register_parser<battery_parser, battery_params>(
      battery_parser::kXMLRoot,
      rcppsw::params::xml_param_parser::kHeader1);
  register_parser<oracle_parser, oracle_params>(
      oracle_parser::kXMLRoot, rcppsw::params::xml_param_parser::kHeader1);
  register_parser<caches::caches_parser,
                  caches::caches_params>(
                      caches::caches_parser::kXMLRoot,
                      rcppsw::params::xml_param_parser::kHeader1);
}

NS_END(params, fordyca);
