/**
 * @file dpo_controller_repository.cpp
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
#include "fordyca/params/depth0/dpo_controller_repository.hpp"
#include "fordyca/params/block_sel_matrix_parser.hpp"
#include "fordyca/params/perception/perception_parser.hpp"

#include "fordyca/params/communication_params.hpp"
#include "fordyca/params/communication_parser.hpp"
/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(fordyca, params, depth0);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dpo_controller_repository::dpo_controller_repository(void) {
  register_parser<block_sel_matrix_parser, block_sel_matrix_params>(
      block_sel_matrix_parser::kXMLRoot, block_sel_matrix_parser::kHeader1);

  register_parser<communication_parser, communication_params>(
      communication_parser::kXMLRoot, communication_parser::kHeader1);

  register_parser<perception::perception_parser, perception::perception_params>(
      perception::perception_parser::kXMLRoot,
      perception::perception_parser::kHeader1);
}

NS_END(depth0, params, fordyca);
