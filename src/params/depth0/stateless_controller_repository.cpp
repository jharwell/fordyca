/**
 * @file stateless_controller_repository.cpp
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
#include "fordyca/params/depth0/stateless_controller_repository.hpp"
#include "rcppsw/control/waveform_xml_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace ct = rcppsw::control;

NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_controller_repository::stateless_controller_repository(void) {
  register_parser<ct::waveform_xml_parser>(std::string("block_carry_") +
                                               ct::waveform_xml_parser::kXMLRoot,
                                           ct::waveform_xml_parser::kHeader1);
}

NS_END(depth0, params, fordyca);
