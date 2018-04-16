/**
 * @file stateless_foraging_repository.cpp
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
#include "fordyca/params/depth0/stateless_foraging_repository.hpp"
#include "fordyca/params/actuation_parser.hpp"
#include "fordyca/params/fsm_parser.hpp"
#include "fordyca/params/output_parser.hpp"
#include "fordyca/params/sensing_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
stateless_foraging_repository::stateless_foraging_repository(void) {
  register_parser<output_parser>(output_parser::kXMLRoot,
                                 output_parser::kHeader1);
  register_parser<actuation_parser>(actuation_parser::kXMLRoot,
                                    actuation_parser::kHeader1);
  register_parser<sensing_parser>(sensing_parser::kXMLRoot,
                                  sensing_parser::kHeader1);
  register_parser<fsm_parser>(fsm_parser::kXMLRoot, fsm_parser::kHeader1);
}

NS_END(depth0, params, fordyca);
