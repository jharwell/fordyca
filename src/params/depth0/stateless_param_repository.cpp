/**
 * @file stateless_param_repository.cpp
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
#include "fordyca/params/depth0/stateless_param_repository.hpp"
#include "fordyca/params/actuation_parser.hpp"
#include "fordyca/params/output_parser.hpp"
#include "fordyca/params/sensing_parser.hpp"
#include "rcppsw/control/waveform_xml_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace ct = rcppsw::control;

NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_param_repository::stateless_param_repository(void) {
  register_parser<output_parser, output_params>(output_parser::kXMLRoot,
                                                output_parser::kHeader1);
  register_parser<actuation_parser, actuation_params>(
      actuation_parser::kXMLRoot, actuation_parser::kHeader1);
  register_parser<sensing_parser, sensing_params>(sensing_parser::kXMLRoot,
                                                  sensing_parser::kHeader1);
  register_parser<energy_parser, energy_params>(energy_parser::kXMLRoot,
                                                energy_parser::kHeader1);
  register_parser<ct::waveform_xml_parser>(std::string("block_carry_") +
                                               ct::waveform_xml_parser::kXMLRoot,
                                           ct::waveform_xml_parser::kHeader1);

}

NS_END(depth0, params, fordyca);
